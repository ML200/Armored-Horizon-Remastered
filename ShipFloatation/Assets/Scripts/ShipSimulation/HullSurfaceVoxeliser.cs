using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Rendering.HighDefinition;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

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

    //Shit for the parallel depth calculation
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

    //DEBUG
    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Vector3 positionWithOffset = transform.position + new Vector3(0, yOffset, 0);
        Gizmos.DrawWireCube(positionWithOffset, volumeSize);

        if (showVoxels)
        {
            foreach (VoxelData data in voxelDataList)
            {
                Gizmos.color = Color.cyan;
                Vector3 globalPosition = transform.TransformPoint(data.Position);
                Quaternion rotation = Quaternion.LookRotation(transform.TransformDirection(data.Normal));
                Gizmos.DrawMesh(plateMesh, globalPosition, rotation, Vector3.one * voxelSize);
                Gizmos.DrawLine(globalPosition, globalPosition + data.Normal);

                // Draw mirrored voxel
                Vector3 globalMirroredPosition = transform.TransformPoint(data.MirroredPosition);
                Quaternion mirroredRotation = Quaternion.LookRotation(transform.TransformDirection(data.MirroredNormal));
                Gizmos.color = Color.green;
                Gizmos.DrawMesh(plateMesh, globalMirroredPosition, mirroredRotation, Vector3.one * voxelSize);
                Gizmos.DrawLine(globalMirroredPosition, globalMirroredPosition + data.MirroredNormal);
            }
        }
    }

    //NODE SIMULATION
    // Start is called before the first frame update
    void Start()
    {
        // Allocate the buffers
        targetPositionBuffer = new NativeArray<float3>(voxelDataList.Count*2, Allocator.Persistent);
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
            targetPositionBuffer[i] = voxelDataList[i * 2].Position;
            targetPositionBuffer[i] = voxelDataList[i * 2+1].Position;
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
        JobHandle handle = searchJob.Schedule(voxelDataList.Count*2, 1);
        handle.Complete();

        // Fill the input positions
        for (int i = 0; i < voxelDataList.Count; ++i)
        {
            Vector3 pos1 = new Vector3(projectedPositionWSBuffer[i*2].x, projectedPositionWSBuffer[i * 2].y, projectedPositionWSBuffer[i * 2].z);
            Vector3 pos2 = new Vector3(projectedPositionWSBuffer[i * 2+1].x, projectedPositionWSBuffer[i * 2+1].y, projectedPositionWSBuffer[i * 2+1].z);
            voxelDataList[i].Depth = (pos1 - voxelDataList[i].Position).magnitude;
            voxelDataList[i].MirrorDepth = (pos2 - voxelDataList[i].Position).magnitude;

        }
        foreach(VoxelData voxelData in voxelDataList)
        {
            float vol = CalculateSubmergedVolume(voxelData);
            float force = fluidDensity * vol * Physics.gravity.magnitude;

            //Calculate the position
            Vector3 midVec = voxelData.Position - voxelData.MirroredPosition;
            Vector3 mid = voxelData.MirroredPosition + 0.5f * midVec;
        }
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


    //VOXEL GENERATION
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
                    ShootRayAndMirror(origin, Vector3.right); // Only from -X to +X
                    if (y == bounds.min.y)
                    {
                        ShootRayAndMirror(origin, Vector3.up); // from bottom to top
                    }

                    if (Time.frameCount % 20 == 0)
                    {
                        yield return null;
                    }
                }
            }
        }
    }

    private void ShootRayAndMirror(Vector3 origin, Vector3 direction)
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
                MirroredNormal = mirroredNormal
            };
            voxelDataList.Add(voxelData);
        }
    }

    private float CalculateSubmergedVolume(VoxelData voxel)
    {
        Vector3 globalPosition = transform.TransformPoint(voxel.Position);
        Vector3 globalMirroredPosition = transform.TransformPoint(voxel.MirroredPosition);

        // Determine the actual submerged depth of each voxel considering their sizes
        float submergedDepth1 = CalculateSubmergedDepth(voxel.Depth);
        float submergedDepth2 = CalculateSubmergedDepth(voxel.MirrorDepth);

        // Calculate average submerged depth
        float averageDepth = (submergedDepth1 + submergedDepth2) / 2;

        // Determine effective submerged length based on the voxel size
        float length = Mathf.Abs(globalPosition.y - globalMirroredPosition.y) + voxelSize;

        // Submerged volume using the cross-sectional area assumed to be voxelSize squared
        float submergedVolume = averageDepth * length * voxelSize;

        return submergedVolume;
    }

    // Helper function to calculate submerged depth considering voxel size
    private float CalculateSubmergedDepth(float depthCenter)
    {
        // Half voxel size above and below the center
        float halfSize = voxelSize / 2;

        // Calculate submerged depth, accounting for voxel size
        if (depthCenter > halfSize)
        {
            // Entire voxel is submerged
            return voxelSize;
        }
        else if (depthCenter > -halfSize)
        {
            // Partially submerged, only the lower portion below the waterline
            return depthCenter + halfSize;
        }
        else
        {
            // Entire voxel is above water
            return 0;
        }
    }
}
public class VoxelData
{
    public Vector3 Position; // Position of the original voxel
    public Vector3 Normal;   // Normal at the original voxel position
    public Vector3 MirroredPosition; // Position of the mirrored voxel
    public Vector3 MirroredNormal;   // Normal at the mirrored voxel position
    public float Depth = 0;       // Submersion depth
    public float MirrorDepth = 0;
}

