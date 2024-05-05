using System.Collections.Generic;
using UnityEngine;

public class ShipSteering : MonoBehaviour
{
    public List<GameObject> screws;
    public float screwMaxAngularSpeed = 1200f;
    public GameObject steeringPoint;
    public GameObject forcePoint;
    public float enginePower = 32000000f;

    private EngineStateManager stateManager = new EngineStateManager();

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.W))
        {
            stateManager.StepUp();
        }
        else if (Input.GetKeyDown(KeyCode.S))
        {
            stateManager.StepDown();
        }
    }

    private void FixedUpdate()
    {
        float power = stateManager.ReturnPowerPercentage() * enginePower;
        GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * power, forcePoint.transform.position);
        foreach (GameObject go in screws)
        {
            go.transform.RotateAround(go.transform.position, go.transform.forward, stateManager.ReturnPowerPercentage() * screwMaxAngularSpeed * Time.fixedDeltaTime);
        }
    }
}

[System.Serializable]
class EngineStateManager
{
    private ENGINE engine = ENGINE.STOP;
    private bool backward = false;

    public float ReturnPowerPercentage()
    {
        float eng = 0;
        switch (engine)
        {
            case ENGINE.QUARTER:
                eng = 0.25f; break;
            case ENGINE.HALF:
                eng = 0.5f; break;
            case ENGINE.FULL:
                eng = 1.0f; break;
        }
        if(!backward)
            return eng;
        else return -eng;
    }
    public void StepUp()
    {
        if(backward && engine == ENGINE.QUARTER)
        {
           engine = ENGINE.STOP;
            backward = false;
        }
        else if (backward && engine != ENGINE.QUARTER)
        {
            switch (engine)
            {
                case ENGINE.HALF:
                    engine = ENGINE.QUARTER;
                    break;
                case ENGINE.FULL:
                    engine = ENGINE.HALF;
                    break;
            }
        }
        else
        {
            switch (engine)
            {
                case ENGINE.HALF:
                    engine = ENGINE.FULL;
                    break;
                case ENGINE.QUARTER:
                    engine = ENGINE.HALF;
                    break;
                case ENGINE.STOP: 
                    engine = ENGINE.QUARTER;
                    break;
            }
        }
    }


    public void StepDown()
    {
        if (!backward && engine == ENGINE.STOP)
        {
            engine = ENGINE.QUARTER;
            backward = true;
        }
        else if (!backward && engine != ENGINE.STOP)
        {
            switch (engine)
            {
                case ENGINE.HALF:
                    engine = ENGINE.QUARTER;
                    break;
                case ENGINE.FULL:
                    engine = ENGINE.HALF;
                    break;
                case ENGINE.QUARTER:
                    engine = ENGINE.STOP; 
                    break;
            }
        }
        else
        {
            switch (engine)
            {
                case ENGINE.HALF:
                    engine = ENGINE.FULL;
                    break;
                case ENGINE.QUARTER:
                    engine = ENGINE.HALF;
                    break;
            }
        }
    }
}


enum ENGINE
{
    FULL,
    HALF,
    QUARTER,
    STOP
}
