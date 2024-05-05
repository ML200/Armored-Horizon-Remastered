using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using System.Drawing;

public class BuoyancyManager : MonoBehaviour
{
    public List<MeshCollider> targetMeshColliders; // List of mesh filters to check against
    public float voxelSize = 1.0f;
    public int gridSizeX = 10;
    public int gridSizeY = 10;
    public int gridSizeZ = 10;
    public bool showVoxels = true;
    private int removedVoxels = 0;
    [HideInInspector]
    public List<Vector3> voxelCenters = new List<Vector3>();

    void OnDrawGizmos()
    {
        // Draw voxel centers if enabled
        if (showVoxels)
        {
            Gizmos.color = UnityEngine.Color.cyan;
            foreach (Vector3 center in voxelCenters)
            {
                Gizmos.DrawSphere(transform.position + center, voxelSize/2.0f);
            }
        }

        // Draw a cube representing the bounds of the grid
        Gizmos.color = UnityEngine.Color.red;
        Vector3 gridCenter = transform.position;
        Vector3 gridSize = new Vector3(gridSizeX * voxelSize, gridSizeY * voxelSize, gridSizeZ * voxelSize);
        Gizmos.DrawWireCube(gridCenter, gridSize);
    }

    public void GenerateVoxelGrid()
    {
        StartCoroutine(GenerateVoxelsAsync());
    }

    private IEnumerator GenerateVoxelsAsync()
    {
        voxelCenters.Clear();
        int totalVoxels = 0;

        for (int x = -gridSizeX / 2; x <= gridSizeX / 2; x++)
        {
            for (int y = -gridSizeY / 2; y <= gridSizeY / 2; y++)
            {
                for (int z = -gridSizeZ / 2; z <= gridSizeZ / 2; z++)
                {
                    Vector3 voxelCenter = transform.position + new Vector3(x * voxelSize, y * voxelSize, z * voxelSize);
                    if (IsInsideAnyMeshFilter(voxelCenter))
                    {
                        voxelCenters.Add(voxelCenter);
                    }
                    totalVoxels++;
                    if (totalVoxels % 100 == 0) // Report progress every 100 voxels
                    {
                        Debug.Log($"Processed {totalVoxels} voxels");
                    }
                    yield return null;
                }
            }
        }
        Debug.Log("Finished processing all voxels.");
        removedVoxels = RemoveAndCountFullySurroundedVoxels();
    }


    // Method to count and remove fully surrounded voxels with direct list comparison
    public int RemoveAndCountFullySurroundedVoxels()
    {
        List<Vector3> toRemove = new List<Vector3>();

        // Convert all voxel centers to Vector3Int for integer comparison
        List<Vector3Int> voxelIndices = new List<Vector3Int>();
        foreach (Vector3 center in voxelCenters)
        {
            voxelIndices.Add(Vector3Int.RoundToInt(center / voxelSize));
        }

        // Check each voxel center against all others to determine if it has exactly six neighbors
        for (int i = 0; i < voxelCenters.Count; i++)
        {
            int neighbors = 0;
            Vector3Int currentVoxel = voxelIndices[i];

            foreach (Vector3Int testVoxel in voxelIndices)
            {
                if (testVoxel != currentVoxel && (
                    (testVoxel == currentVoxel + Vector3Int.right) ||
                    (testVoxel == currentVoxel - Vector3Int.right) ||
                    (testVoxel == currentVoxel + Vector3Int.up) ||
                    (testVoxel == currentVoxel - Vector3Int.up) ||
                    (testVoxel == currentVoxel + new Vector3Int(0, 0, 1)) ||
                    (testVoxel == currentVoxel - new Vector3Int(0, 0, 1))
                    ))
                {
                    neighbors++;
                }
            }

            // If exactly six neighbors, schedule for removal
            if (neighbors == 6)
            {
                toRemove.Add(voxelCenters[i]);
            }
        }

        // Remove fully surrounded voxels all at once
        foreach (Vector3 remove in toRemove)
        {
            voxelCenters.Remove(remove);
        }

        return toRemove.Count;
    }

    public List<Vector3> RotateVoxelCenters(Quaternion rotation)
    {
        List<Vector3> rotatedVoxels = new List<Vector3>();
        foreach (Vector3 center in voxelCenters)
        {
            rotatedVoxels.Add(rotation * center); // Apply the rotation to each center
        }
        return rotatedVoxels;
    }

    private bool IsInsideAnyMeshFilter(Vector3 point)
    {
        foreach (var meshColl in targetMeshColliders)
        {
            if (IsPointInsideCollider(point + transform.position, meshColl))
            {
                return true;
            }
        }
        return false;
    }

    public static bool IsPointInsideCollider(Vector3 position, MeshCollider collider)
    {
        // Check if a point is inside a mesh using Physics.CheckBox
        return Physics.CheckBox(position, new Vector3(0.01f, 0.01f, 0.01f), Quaternion.identity, LayerMask.GetMask("Default"));
    }
}
