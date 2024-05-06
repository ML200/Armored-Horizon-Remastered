using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class FoamDeformation : MonoBehaviour
{
    public Transform target;
    private WaterSurface water;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        water = GetComponent<WaterSurface>();
    }

    // Update is called once per frame
    void Update()
    {
        water.foamAreaOffset = target.position;
        water.deformationAreaOffset = target.position;
    }
}
