using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Rendering.HighDefinition;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.VisualScripting;

public class HullSurfaceVoxelizer : MonoBehaviour
{
    public float voxelSize = 1.0f;
    public float density = 10.0f; // Rays per unit distance
    public Vector3 volumeSize = new Vector3(10, 5, 10); // Volume size for raycasting
    public float yOffset = 0.0f; // Y Offset for the scanning volume
    public bool showVoxels = true;
    public List<VoxelData> voxelDataList = new List<VoxelData>();
    public LayerMask collisionLayer; // Specify which layers to detect
    public Mesh plateMesh;
    public float fluidDensity = 1000.0f; // Density of water: 1000 kg/m³
    public float dragModifier = 1.0f;
    public float volumeModifier = 1.0f;
    public float depthModifier = 1.0f;
    public float averagingRadius = 0.0f; // Radius for depth averaging
    public float nonSurfaceRadiusModifier = 1.0f; // Radius modifier for non-surface voxels

    private Dictionary<Vector2Int, List<VoxelData>> voxelGrid = new Dictionary<Vector2Int, List<VoxelData>>();
    private List<VoxelData> nonSurfaceVoxels = new List<VoxelData>();

    // Shit for the parallel depth calculation
    // Public parameter
    public WaterSurface waterSurface;
    // Input job parameters
    NativeArray<float3> targetPositionBuffer;
    // Output job parameters
    NativeArray<float> errorBuffer;
    NativeArray<float3> candidatePositionBuffer;
    NativeArray<float3> projectedPositionWSBuffer;
    NativeArray<float3> normalWSBuffer;
    NativeArray<float3> directionBuffer;
    NativeArray<int> stepCountBuffer;

    // DEBUG
    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Vector3 positionWithOffset = transform.position + new Vector3(0, yOffset, 0);
        Gizmos.DrawWireCube(positionWithOffset, volumeSize);

        if (showVoxels)
        {
            foreach (VoxelData data in voxelDataList)
            {
                Vector3 globalPosition = transform.TransformPoint(data.Position);
                Vector3 globalMirroredPosition = transform.TransformPoint(data.MirroredPosition);

                // Draw voxel and mirrored voxel with their normals
                Gizmos.color = Color.cyan;
                Quaternion rotation = Quaternion.LookRotation(transform.TransformDirection(data.Normal));
                //Gizmos.DrawMesh(plateMesh, globalPosition, rotation, Vector3.one * voxelSize);
                //Gizmos.DrawLine(globalPosition, globalPosition + transform.TransformVector(data.Normal));

                Quaternion mirroredRotation = Quaternion.LookRotation(transform.TransformDirection(data.MirroredNormal));
                Gizmos.color = Color.green;
                //Gizmos.DrawMesh(plateMesh, globalMirroredPosition, mirroredRotation, Vector3.one * voxelSize);
                //Gizmos.DrawLine(globalMirroredPosition, globalMirroredPosition + transform.TransformVector(data.MirroredNormal));

                (float vol, Vector3 centroid) = CalculateSubmergedVolume(data);
                float maxVol = voxelSize * voxelSize * (data.Position - data.MirroredPosition).magnitude;

                // Interpolate color from red (empty) to blue (full)
                Color volumeColor = Color.Lerp(Color.red, Color.blue, vol / maxVol);
                Gizmos.color = volumeColor;

                // Determine the size and position of the cuboid
                Vector3 midPoint = (globalPosition + globalMirroredPosition) / 2;
                float distance = Vector3.Distance(globalPosition, globalMirroredPosition);
                Vector3 cuboidSize = new Vector3(voxelSize, voxelSize, distance);
                Quaternion cuboidRotation = Quaternion.LookRotation(globalMirroredPosition - globalPosition);

                // Draw volume cuboid
                Gizmos.matrix = Matrix4x4.TRS(midPoint, cuboidRotation, cuboidSize);
                if (data.isVolume)
                    Gizmos.DrawCube(Vector3.zero, Vector3.one);  // Draw cuboid at the local origin with size 1 (scale is handled by matrix)
                Gizmos.matrix = Matrix4x4.identity; // Reset matrix to default to avoid affecting other Gizmos
            }
        }
    }

    // NODE SIMULATION
    // Start is called before the first frame update
    void Start()
    {
        // Allocate the buffers
        targetPositionBuffer = new NativeArray<float3>(voxelDataList.Count * 2, Allocator.Persistent);
        errorBuffer = new NativeArray<float>(voxelDataList.Count * 2, Allocator.Persistent);
        candidatePositionBuffer = new NativeArray<float3>(voxelDataList.Count * 2, Allocator.Persistent);
        projectedPositionWSBuffer = new NativeArray<float3>(voxelDataList.Count * 2, Allocator.Persistent);
        normalWSBuffer = new NativeArray<float3>(voxelDataList.Count * 2, Allocator.Persistent);
        directionBuffer = new NativeArray<float3>(voxelDataList.Count * 2, Allocator.Persistent);
        stepCountBuffer = new NativeArray<int>(voxelDataList.Count * 2, Allocator.Persistent);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (waterSurface == null)
            return;
        // Try to get the simulation data if available
        WaterSimSearchData simData = new WaterSimSearchData();
        if (!waterSurface.FillWaterSearchData(ref simData))
            return;

        // Fill the input positions
        for (int i = 0; i < voxelDataList.Count; ++i)
        {
            Vector3 globalPosition = transform.TransformPoint(voxelDataList[i].Position);
            Vector3 globalMirroredPosition = transform.TransformPoint(voxelDataList[i].MirroredPosition);

            targetPositionBuffer[2 * i] = globalPosition;
            targetPositionBuffer[2 * i + 1] = globalMirroredPosition;
            //Debug.Log(globalPosition + "    " + targetPositionBuffer[2 * i]);
        }

        // Prepare the first band
        WaterSimulationSearchJob searchJob = new WaterSimulationSearchJob();

        // Assign the simulation data
        searchJob.simSearchData = simData;

        // Fill the input data
        searchJob.targetPositionWSBuffer = targetPositionBuffer;
        searchJob.startPositionWSBuffer = targetPositionBuffer;
        searchJob.maxIterations = 8;
        searchJob.error = 0.01f;
        searchJob.includeDeformation = true;
        searchJob.excludeSimulation = false;

        searchJob.errorBuffer = errorBuffer;
        searchJob.candidateLocationWSBuffer = candidatePositionBuffer;
        searchJob.projectedPositionWSBuffer = projectedPositionWSBuffer;
        searchJob.normalWSBuffer = normalWSBuffer;
        searchJob.directionBuffer = directionBuffer;
        searchJob.stepCountBuffer = stepCountBuffer;

        // Schedule the job with one Execute per index in the results array and only 1 item per processing batch
        JobHandle handle = searchJob.Schedule(voxelDataList.Count * 2, 1);
        handle.Complete();

        // Fill the input positions
        for (int i = 0; i < voxelDataList.Count; ++i)
        {
            Vector3 pos1 = new Vector3(projectedPositionWSBuffer[i * 2].x, projectedPositionWSBuffer[i * 2].y, projectedPositionWSBuffer[i * 2].z);
            Vector3 pos2 = new Vector3(projectedPositionWSBuffer[i * 2 + 1].x, projectedPositionWSBuffer[i * 2 + 1].y, projectedPositionWSBuffer[i * 2 + 1].z);
            voxelDataList[i].Depth = (pos1 - transform.TransformPoint(voxelDataList[i].Position)).y;
            voxelDataList[i].MirrorDepth = (pos2 - transform.TransformPoint(voxelDataList[i].MirroredPosition)).y;
            voxelDataList[i].Direction = directionBuffer[i * 2];
            voxelDataList[i].MirrorDirection = directionBuffer[i * 2];
        }

        // Create a grid for easier neighbor lookup
        CreateVoxelGrid();

        foreach (VoxelData voxelData in voxelDataList)
        {
            (float vol, Vector3 centroid) = CalculateSubmergedVolume(voxelData);
            (float s1, float s2) = CalculateSubmergedArea(voxelData);
            Vector3 force = fluidDensity * vol * -Physics.gravity;

            Vector3 vRig1 = GetComponent<Rigidbody>().GetRelativePointVelocity(voxelData.Position);
            Vector3 vRig2 = GetComponent<Rigidbody>().GetRelativePointVelocity(voxelData.MirroredPosition);
            Vector3 drag1 = CalculateDragForce(s1, voxelData.Direction.normalized * waterSurface.largeCurrentSpeedValue, vRig1, -voxelData.Normal);
            Vector3 drag2 = CalculateDragForce(s2, voxelData.Direction.normalized * waterSurface.largeCurrentSpeedValue, vRig2, -voxelData.MirroredNormal);
            Debug.DrawLine(centroid, centroid + force * 0.0001f, Color.blue);
            //Debug.DrawLine(transform.TransformPoint(voxelData.Position), transform.TransformPoint(voxelData.Position) + voxelData.Depth * Vector3.up, Color.white);
            //Debug.DrawLine(transform.TransformPoint(voxelData.MirroredPosition), transform.TransformPoint(voxelData.MirroredPosition) + voxelData.MirrorDepth * Vector3.up, Color.white);
            //Debug.DrawLine(transform.TransformPoint(voxelData.Position), transform.TransformPoint(voxelData.Position) + voxelData.Direction);
            //Debug.DrawLine(transform.TransformPoint(voxelData.MirroredPosition), transform.TransformPoint(voxelData.MirroredPosition) + voxelData.MirrorDirection);
            Debug.DrawLine(transform.TransformPoint(voxelData.Position), transform.TransformPoint(voxelData.Position) - drag1 * 0.0001f);
            Debug.DrawLine(transform.TransformPoint(voxelData.MirroredPosition), transform.TransformPoint(voxelData.MirroredPosition) - drag2 * 0.0001f);
            GetComponent<Rigidbody>().AddForceAtPosition(-drag1, voxelData.Position);
            GetComponent<Rigidbody>().AddForceAtPosition(-drag2, voxelData.MirroredPosition);
            GetComponent<Rigidbody>().AddForceAtPosition(force * volumeModifier, centroid);
        }

        foreach (VoxelData nonSurfaceVoxel in nonSurfaceVoxels)
        {
            (float vol, Vector3 centroid) = CalculateSubmergedVolume(nonSurfaceVoxel, true);
            Vector3 force = fluidDensity * vol * -Physics.gravity;

            GetComponent<Rigidbody>().AddForceAtPosition(force * volumeModifier, centroid);
        }
    }

    private void CreateVoxelGrid()
    {
        voxelGrid.Clear();
        nonSurfaceVoxels.Clear();
        foreach (VoxelData voxel in voxelDataList)
        {
            Vector2Int gridPos = new Vector2Int(Mathf.FloorToInt(voxel.Position.x / voxelSize), Mathf.FloorToInt(voxel.Position.z / voxelSize));
            if (!voxelGrid.ContainsKey(gridPos))
            {
                voxelGrid[gridPos] = new List<VoxelData>();
            }
            voxelGrid[gridPos].Add(voxel);

            if (voxel.isVolume)
            {
                VoxelData nonSurfaceVoxel = new VoxelData
                {
                    Position = voxel.Position,
                    Normal = voxel.Normal,
                    MirroredPosition = voxel.MirroredPosition,
                    MirroredNormal = voxel.MirroredNormal,
                    Depth = voxel.Depth,
                    MirrorDepth = voxel.MirrorDepth,
                    Direction = voxel.Direction,
                    MirrorDirection = voxel.MirrorDirection,
                    isVolume = false // Non-surface voxel
                };
                nonSurfaceVoxels.Add(nonSurfaceVoxel);
            }
        }
    }

    private List<VoxelData> GetNeighborVoxels(VoxelData voxel, float radius, bool mirrored)
    {
        List<VoxelData> neighbors = new List<VoxelData>();
        Vector2Int gridPos = new Vector2Int(Mathf.FloorToInt(voxel.Position.x / voxelSize), Mathf.FloorToInt(voxel.Position.z / voxelSize));

        foreach (var offset in GetOffsetsWithinRadius(radius))
        {
            Vector2Int neighborGridPos = gridPos + offset;
            if (voxelGrid.ContainsKey(neighborGridPos))
            {
                foreach (VoxelData neighbor in voxelGrid[neighborGridPos])
                {
                    if (mirrored && neighbor.isVolume || !mirrored && !neighbor.isVolume)
                    {
                        neighbors.Add(neighbor);
                    }
                }
            }
        }
        return neighbors;
    }

    private List<Vector2Int> GetOffsetsWithinRadius(float radius)
    {
        List<Vector2Int> offsets = new List<Vector2Int>();
        int steps = Mathf.CeilToInt(radius / voxelSize);

        for (int x = -steps; x <= steps; x++)
        {
            for (int z = -steps; z <= steps; z++)
            {
                if (x * x + z * z <= steps * steps)
                {
                    offsets.Add(new Vector2Int(x, z));
                }
            }
        }
        return offsets;
    }

    // DRAG HELPERS
    // Calculate the drag force on a plate
    public Vector3 CalculateDragForce(float area, Vector3 vw, Vector3 vs, Vector3 normal)
    {
        // Calculate relative velocity
        Vector3 vRel = vw - vs;

        // Compute the magnitude of the relative velocity
        float speedRel = vRel.magnitude;

        // Calculate angle between the normal to the plate and the relative velocity
        float cosTheta = Vector3.Dot(normal, vRel.normalized);

        // Calculate effective area
        float effectiveArea = area * cosTheta; // Only consider positive cosTheta for facing the flow

        // Calculate drag coefficient using the quadratic approximation
        float thetaDegrees = Mathf.Acos(Mathf.Clamp(cosTheta, -1.0f, 1.0f)) * Mathf.Rad2Deg;
        float CD = CalculateDragCoefficient(thetaDegrees);
        float factor = cosTheta > 0 ? 1.0f : -1.0f;
        // Calculate drag force magnitude
        float dragForceMagnitude = 0.5f * fluidDensity * CD * effectiveArea * speedRel * speedRel * dragModifier * factor;

        // Direction of drag force is opposite to the relative velocity
        Vector3 dragForce = -dragForceMagnitude * vRel.normalized;

        return dragForce;
    }

    // Drag coefficient approximation as a function of angle
    private float CalculateDragCoefficient(float theta)
    {
        // Coefficients derived earlier
        float a = 0.000242f;
        float b = -0.04378f;
        float c = 1.98f;

        // Compute CD based on angle
        return a * theta * theta + b * theta + c;
    }

    private void OnDestroy()
    {
        targetPositionBuffer.Dispose();
        errorBuffer.Dispose();
        candidatePositionBuffer.Dispose();
        projectedPositionWSBuffer.Dispose();
        normalWSBuffer.Dispose();
        directionBuffer.Dispose();
        stepCountBuffer.Dispose();
    }

    // VOXEL GENERATION
    public void StartVoxelGeneration()
    {
        StartCoroutine(GenerateSurfaceVoxels());
    }

    private IEnumerator GenerateSurfaceVoxels()
    {
        voxelDataList.Clear();
        Vector3 centerWithOffset = transform.position + new Vector3(0, yOffset, 0);
        Bounds bounds = new Bounds(centerWithOffset, volumeSize);

        float stepX = 1 / density;
        float stepY = 1 / density;
        float stepZ = 1 / density;

        for (float x = bounds.min.x; x <= bounds.center.x; x += stepX)
        {
            for (float y = bounds.min.y; y <= bounds.max.y; y += stepY)
            {
                for (float z = bounds.min.z; z <= bounds.max.z; z += stepZ)
                {
                    Vector3 origin = new Vector3(x, y, z);
                    ShootRayAndMirror(origin, Vector3.right, true); // Only from -X to +X
                    if (y == bounds.min.y)
                    {
                        ShootRayAndMirror(origin, Vector3.up, false); // from bottom to top
                    }

                    if (Time.frameCount % 20 == 0)
                    {
                        yield return null;
                    }
                }
            }
        }
    }

    private void ShootRayAndMirror(Vector3 origin, Vector3 direction, bool volume)
    {
        RaycastHit hit;
        if (Physics.Raycast(origin, direction, out hit, Mathf.Infinity, collisionLayer))
        {
            Vector3 localHitPoint = transform.InverseTransformPoint(hit.point);
            Vector3 localHitNormal = transform.InverseTransformDirection(hit.normal);

            // Create mirrored voxel data
            Vector3 mirroredPoint = new Vector3(-localHitPoint.x, localHitPoint.y, localHitPoint.z);
            Vector3 mirroredNormal = new Vector3(-localHitNormal.x, localHitNormal.y, localHitNormal.z);

            VoxelData voxelData = new VoxelData
            {
                Position = localHitPoint,
                Normal = localHitNormal,
                MirroredPosition = mirroredPoint,
                MirroredNormal = mirroredNormal,
                isVolume = volume
            };
            voxelDataList.Add(voxelData);
        }
    }

    private (float s1, float s2) CalculateSubmergedArea(VoxelData voxel)
    {
        float submergedDepth1 = voxel.Depth > 0 ? 1.0f : 0.0f;
        float submergedDepth2 = voxel.MirrorDepth > 0 ? 1.0f : 0.0f;

        return (submergedDepth1 * voxelSize * voxelSize, submergedDepth2 * voxelSize * voxelSize);
    }

    private (float submergedVolume, Vector3 centroid) CalculateSubmergedVolume(VoxelData voxel, bool isNonSurfaceVoxel = false)
    {
        if (!voxel.isVolume && !isNonSurfaceVoxel)
        {
            return (0.0f, new Vector3());
        }

        Vector3 globalPosition = transform.TransformPoint(voxel.Position);
        Vector3 globalMirroredPosition = transform.TransformPoint(voxel.MirroredPosition);

        // Get the neighboring voxels
        float radius = isNonSurfaceVoxel ? averagingRadius * nonSurfaceRadiusModifier : averagingRadius;
        List<VoxelData> neighbors = GetNeighborVoxels(voxel, radius, voxel.isVolume);

        // Calculate the average depth
        float avgDepth = 0;
        float avgMirrorDepth = 0;
        int count = 0;
        foreach (VoxelData neighbor in neighbors)
        {
            if (neighbor.Depth > 0 || neighbor.MirrorDepth > 0) // Only consider subsurface voxels
            {
                avgDepth += neighbor.Depth;
                avgMirrorDepth += neighbor.MirrorDepth;
                count++;
            }
        }

        if (count > 0)
        {
            avgDepth /= count;
            avgMirrorDepth /= count;
        }
        else
        {
            avgDepth = voxel.Depth;
            avgMirrorDepth = voxel.MirrorDepth;
        }

        // Assuming that voxelSize is the dimension along the shortest side (width/height) and length is the longest side.
        float length = (voxel.Position - voxel.MirroredPosition).magnitude;
        Vector3 direction = (globalPosition - globalMirroredPosition).normalized;
        float rotFactor = Mathf.Abs(Vector3.Dot(direction, Vector3.up));

        // Size adjusts from voxelSize to length based on the rotation factor.
        float size = Mathf.Lerp(voxelSize, length, rotFactor);

        // Calculate submerged depths considering the interpolated size.
        float submergedDepth1 = CalculateSubmergedDepth(avgDepth, size, rotFactor);
        float submergedDepth2 = CalculateSubmergedDepth(avgMirrorDepth, size, rotFactor);

        // Adjusted calculations
        float v1 = 0;
        float v2 = 0;

        if (submergedDepth1 > 0 || submergedDepth2 > 0)
        {
            float totalDepth = submergedDepth1 + submergedDepth2;
            if (voxel.Depth >= voxel.MirrorDepth)
            {
                float r = Mathf.Abs(submergedDepth1) / totalDepth;
                r = Mathf.Lerp(0.5f, r, depthModifier); // Smoothens the ratio towards an even split based on the depthModifier
                v1 = submergedDepth1 * r;
                v2 = submergedDepth2 * (1 - r);
            }
            else
            {
                float r = Mathf.Abs(submergedDepth2) / totalDepth;
                r = Mathf.Lerp(0.5f, r, depthModifier); // Smoothens the ratio towards an even split based on the depthModifier
                v2 = submergedDepth2 * r;
                v1 = submergedDepth1 * (1 - r);
            }
        }

        // Calculate submerged volume using adjusted sizes and depths.
        float submergedVolume = (v1 + v2) * length * voxelSize * voxelSize / 2;

        // Calculate the centroid
        Vector3 midDir = globalPosition - globalMirroredPosition;
        Vector3 centroid = new Vector3();
        if (v2 != 0 || v1 != 0)
        {
            centroid = v1 / (v1 + v2) * midDir + globalMirroredPosition;
        }
        else
        {
            centroid = 0.5f * midDir + globalMirroredPosition;
        }

        return (submergedVolume, centroid);
    }

    // Helper function to calculate submerged depth considering voxel size
    private float CalculateSubmergedDepth(float depthCenter, float size, float sizeFactor)
    {
        // Half voxel size above and below the center
        float halfSize = size / 2;
        if (depthCenter >= 0)
        {
            // Calculate submerged depth, accounting for voxel size
            if (depthCenter > halfSize + halfSize * sizeFactor)
            {
                // Entire voxel is submerged
                return size;
            }
            else if (depthCenter < halfSize + halfSize * sizeFactor)
            {
                // Partially submerged, only the lower portion below the waterline
                return depthCenter + halfSize + halfSize * sizeFactor;
            }
            else
            {
                // Half voxel is above water
                return halfSize + halfSize * sizeFactor;
            }
        }
        else
        {
            depthCenter = -depthCenter;
            if (depthCenter > halfSize + halfSize * sizeFactor)
            {
                return 0;
            }
            else if (depthCenter < halfSize + halfSize * sizeFactor)
            {
                return halfSize + halfSize * sizeFactor - depthCenter;
            }
            else
            {
                return halfSize + halfSize * sizeFactor;
            }
        }
    }
}

[System.Serializable]
public class VoxelData
{
    public Vector3 Position; // Position of the original voxel
    public Vector3 Normal;   // Normal at the original voxel position
    public Vector3 MirroredPosition; // Position of the mirrored voxel
    public Vector3 MirroredNormal;   // Normal at the mirrored voxel position
    public float Depth = 0;       // Submersion depth
    public float MirrorDepth = 0;
    public Vector3 Direction = new Vector3();
    public Vector3 MirrorDirection = new Vector3();
    public bool isVolume = false;
}
