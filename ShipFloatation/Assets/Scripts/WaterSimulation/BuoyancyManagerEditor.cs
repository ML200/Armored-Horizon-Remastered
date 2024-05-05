using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(BuoyancyManager))]
public class BuoyancyManagerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector(); // Draws the default inspector

        BuoyancyManager script = (BuoyancyManager)target;

        if (GUILayout.Button("Generate Voxels"))
        {
            script.GenerateVoxelGrid();
        }

        if (GUILayout.Button("Clear Voxels"))
        {
            script.voxelCenters.Clear(); // Clear the voxel list
        }

        script.showVoxels = GUILayout.Toggle(script.showVoxels, "Show Voxels");
    }
}
