using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(HullSurfaceVoxelizer))]
public class HullSurfaceVoxelizerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector(); // Draws the default inspector

        HullSurfaceVoxelizer script = (HullSurfaceVoxelizer)target; // Get the target object

        if (GUILayout.Button("Generate Voxels")) // Add a button to the inspector
        {
            script.StartVoxelGeneration(); // Call the method to generate voxels when the button is pressed
        }
    }
}
