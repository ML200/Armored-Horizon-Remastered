using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class BuoyancyObject : MonoBehaviour
{
    public float buoyancyModifier = 1.0f;
    public Vector3 translationalDragModifier = new Vector3(1.0f, 0.5f, 0.5f);
    public float angularDragModifier = 1.0f;

    public WaterSurface targetSurface = null;
    // Internal search params
    WaterSearchParameters searchParameters = new WaterSearchParameters();
    WaterSearchResult searchResult = new WaterSearchResult();

    private void FixedUpdate()
    {
        if (targetSurface != null)
        {
            Rigidbody rb = GetComponent<Rigidbody>(); // Get the Rigidbody component
            List<Vector3> voxels = GetComponentInChildren<BuoyancyManager>().RotateVoxelCenters(transform.rotation);
            Vector3 voxelManagerPos = GetComponentInChildren<BuoyancyManager>().transform.position;
            float voxelRadius = GetComponentInChildren<BuoyancyManager>().voxelSize / 2.0f;
            float volume = (4.0f / 3.0f) * Mathf.PI * Mathf.Pow(voxelRadius, 3);
            float surfaceArea = Mathf.PI * voxelRadius * voxelRadius;

            // Density of the fluid, gravitational constant, and drag coefficient
            float fluidDensity = 1000.0f; // Water density in kg/m^3
            float gravity = 9.81f; // Gravity in m/s^2
            float dragCoefficient = 0.47f; // Drag coefficient for a sphere

            foreach (var voxel in voxels)
            {
                searchParameters.targetPositionWS = voxel + voxelManagerPos;
                if (targetSurface.ProjectPointOnWaterSurface(searchParameters, out searchResult))
                {
                    float depthBelowSurface = searchResult.projectedPositionWS.y - (voxel.y + voxelManagerPos.y);
                    float submergedVolume = CalculateSubmergedVolume(depthBelowSurface, voxelRadius, volume);

                    // Calculate local velocity at voxel
                    Vector3 r = (voxel + voxelManagerPos) - rb.worldCenterOfMass;
                    Vector3 globalVelocity = rb.linearVelocity;
                    Vector3 localVelocity = transform.InverseTransformDirection(globalVelocity); // Convert global to local
                    Vector3 rotationalVelocity = Vector3.Cross(rb.angularVelocity, r);

                    // Apply modifiers to each component of the local translational velocity
                    localVelocity = new Vector3(
                        localVelocity.x * translationalDragModifier.x,
                        localVelocity.y * translationalDragModifier.y,
                        localVelocity.z * translationalDragModifier.z
                    );

                    // Re-transform to global space for accurate force application
                    localVelocity = transform.TransformDirection(localVelocity);
                    rotationalVelocity *= angularDragModifier; // Apply angular drag modifier

                    Vector3 totalVelocity = localVelocity + rotationalVelocity;
                    float speed = totalVelocity.magnitude;
                    float waterDrag = submergedVolume > 0 ? 1 : 0;
                    // Calculate forces
                    Vector3 buoyantForceVector = Vector3.up * fluidDensity * gravity * volume/*submergedVolume*/ * buoyancyModifier * waterDrag;
                    Vector3 dragForceVector = -0.5f * dragCoefficient * fluidDensity * surfaceArea * speed * waterDrag * totalVelocity.normalized;

                    // Apply forces
                    rb.AddForceAtPosition(buoyantForceVector + dragForceVector, voxel + voxelManagerPos);
                }
            }
        }
    }

    private float CalculateSubmergedVolume(float depthBelowSurface, float voxelRadius, float volume)
    {
        if (depthBelowSurface >= voxelRadius)
            return volume; // Fully submerged
        else if (depthBelowSurface > 0)
        {
            float h = voxelRadius + depthBelowSurface;
            return (Mathf.PI * h * h / 3) * (3 * voxelRadius - h); // Volume of spherical cap
        }
        return 0; // Above the surface
    }


}