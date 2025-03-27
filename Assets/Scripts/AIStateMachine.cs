using UnityEngine;
using System.Collections.Generic;

//parallel state machine logic
public class AIStateMachine : MonoBehaviour
{
    public Vector3 destination = new Vector3(-30, 70f, 750f);
    public bool SidewaysProximityStateOn = true;
    private IAIState currentState;
    private List<IAIState> activeStates = new List<IAIState>();

    private void Start()
    {
        AddState(new BestPathForwardState());
        AddState(new SidewaysProximityState());
    }

    private void Update()
    {
        foreach (var state in activeStates)
        {
            state.Execute(this);
        }
    }

    public void AddState(IAIState state)
    {
        activeStates.Add(state);
    }

    public void RemoveState(IAIState state)
    {
        activeStates.Remove(state);
    }
}

public interface IAIState
{
    void Execute(AIStateMachine ai);
}

//state for moving forward and yawing away from obstacles
public class BestPathForwardState : IAIState
{
    public float moveSpeed = 10f;
    public float spherecastWidth = 3f;
    public float fovAngle = 170f;
    public int rayCount = 300;
    public float rayDistance = 70f;

    public void Execute(AIStateMachine ai)
    {
        //dont want destination to be affected by height
        ai.destination.y = ai.transform.position.y;

        Vector3 bestDirection = FindBestPath(ai);
        
        Debug.Log($"Best Direction: {bestDirection}");

        //find a path forward else go up
        if(bestDirection != Vector3.up)
        {
            ai.SidewaysProximityStateOn = true;

            AIHelper.ControlYaw(ai, bestDirection);
            AIHelper.ControlPitch(ai, true);

            //next four lines of code work to calculate speed based on pitch like a real drone
            float xRotation = ai.transform.rotation.eulerAngles.x;

            float speedMultiplier = Mathf.Abs(xRotation) / 8f;

            float adjustedSpeed = moveSpeed * speedMultiplier * speedMultiplier;

            Vector3 flatForward = new Vector3(ai.transform.forward.x, 0, ai.transform.forward.z).normalized;

            ai.transform.position += flatForward * adjustedSpeed * Time.deltaTime;

        }
        else
        {
            //no roll control needed
            ai.SidewaysProximityStateOn = false;

            AIHelper.ControlYaw(ai, ai.destination);
            AIHelper.ControlPitch(ai, false);
            
            ai.transform.position += ai.transform.up * moveSpeed * Time.deltaTime; 
        }
    }

    Vector3 FindBestPath(AIStateMachine ai)
    {
        float angleStep = fovAngle / rayCount; 
        //default to up if no other options
        Vector3 bestDirection = Vector3.up;      
        //aim directly towards target      
        Vector3 directionToTarget = AIHelper.GetDirectionToTarget(ai.transform.position, ai.destination);        

        for (int i = 0; i < rayCount; i++)
        {
            //fan out from center rather than prioritizing one direction first
            float angle = (i % 2 == 0 ? 1 : -1) * (i / 2) * angleStep;

            Vector3 direction = Quaternion.Euler(0, angle, 0) * directionToTarget;

            //slightly move ray cast up might help with bugs
            Vector3 origin = ai.transform.position + Vector3.up * 0.5f + Vector3.forward * 3f; 

            if (!Physics.SphereCast(origin, spherecastWidth, direction, out _, rayDistance))
            {
                bestDirection = direction;
                Debug.DrawRay(origin, direction * rayDistance, Color.green);
                break;
            }
            else
            {
                Debug.DrawRay(origin, direction * rayDistance, Color.red);
            }
        }

        return bestDirection;
    }

}

public class SidewaysProximityState : IAIState
{
    public float rayDistance = 60f;
    public float spherecastWidth = .5f;

    public float sidewaysMoveSpeed = 3.5f;

    public float forwardAngle = 75f;

    public void Execute(AIStateMachine ai)
    {
        if(!ai.SidewaysProximityStateOn)
            return;

        Vector3 direction = new Vector3(0, 0, 0);
        
        MoveLeftOrRight(ai);

        float zRotation = ai.transform.rotation.eulerAngles.z;

        if(zRotation > 0)
            direction = Vector3.Cross(ai.transform.forward, Vector3.up).normalized;

        if (zRotation > 180)
        {
            direction = Vector3.Cross(Vector3.up, ai.transform.forward).normalized;
            zRotation -= 360; 
        }

        float speedMultiplier = Mathf.Abs(zRotation) / 13f;

        float adjustedSpeed = sidewaysMoveSpeed * speedMultiplier * speedMultiplier;

        ai.transform.position += direction * adjustedSpeed * Time.deltaTime;
    }

    void MoveLeftOrRight(AIStateMachine ai)
    {
        RaycastHit leftHit, rightHit;
        Vector3 rightOrigin = ai.transform.position + Vector3.up * 0.5f + Vector3.forward * 3f + Vector3.right * 2f; 
        Vector3 leftOrigin = ai.transform.position + Vector3.up * 0.5f + Vector3.forward * 3f + Vector3.left * 2f; 


        Vector3 leftDirection = Vector3.Cross(ai.transform.forward, Vector3.up).normalized;
        Vector3 rightDirection = Vector3.Cross(Vector3.up, ai.transform.forward).normalized;

        Vector3 forwardLeftDirection = Quaternion.Euler(0, forwardAngle, 0) * leftDirection;
        Vector3 forwardRightDirection = Quaternion.Euler(0, -1f * forwardAngle, 0) * rightDirection;

        Physics.SphereCast(leftOrigin, spherecastWidth, forwardLeftDirection, out leftHit, rayDistance);
        Physics.SphereCast(rightOrigin, spherecastWidth, forwardRightDirection, out rightHit, rayDistance);

        float leftDistance = leftHit.distance;
        float rightDistance = rightHit.distance;

        if(leftDistance == rightDistance)       //no hits
        {
            AIHelper.ControlRoll(ai, true, true);
        }
        else if(leftDistance == 0)      //right hit
        {
            Debug.DrawRay(rightOrigin, forwardRightDirection * rightDistance, Color.cyan, .05f);

            AIHelper.ControlRoll(ai, true, false);
        }
        else if(rightDistance == 0)     //left hit
        {
            Debug.DrawRay(leftOrigin, forwardLeftDirection * leftDistance, Color.cyan, .05f);

            AIHelper.ControlRoll(ai, false, false);
        }
        else        //both hit
        {
            Debug.DrawRay(leftOrigin, forwardLeftDirection * leftDistance, Color.cyan, .05f);
            Debug.DrawRay(rightOrigin, forwardRightDirection * rightDistance, Color.cyan, .05f);
            
            AIHelper.ControlRoll(ai, leftDistance > rightDistance ? true : false, false);
        }
    }
}


public static class AIHelper
{
    public static Vector3 GetDirectionToTarget(Vector3 position, Vector3 destination)
    {
        return (destination - position).normalized;
    }

    public static void ControlYaw(AIStateMachine ai, Vector3 targetDirection)
    {
        float rotationSpeed = 1f;

        Quaternion targetRotation = Quaternion.LookRotation(targetDirection);

         Quaternion newRotation = Quaternion.Euler(ai.transform.rotation.eulerAngles.x, targetRotation.eulerAngles.y, ai.transform.rotation.eulerAngles.z);

        ai.transform.rotation = Quaternion.Slerp(ai.transform.rotation, newRotation, Time.deltaTime * rotationSpeed);
    }

    public static void ControlRoll(AIStateMachine ai, bool rollRight, bool cancel)
    {
        float rotationSpeed = 1f;
        float tiltAngle = 13f; 

        Quaternion baseRotation = Quaternion.LookRotation(ai.transform.forward, Vector3.up);

        float angle = rollRight ? tiltAngle : -tiltAngle;

        if(cancel)
            angle = 0f;

        Quaternion targetRotation = Quaternion.AngleAxis(angle, ai.transform.forward) * baseRotation;

        ai.transform.rotation = Quaternion.Slerp(ai.transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);
    }

    public static void ControlPitch(AIStateMachine ai, bool pitchForward)
    {
        float rotationSpeed = .5f;
        float tiltAngle = 8f; 

        Vector3 currentRotation = ai.transform.rotation.eulerAngles;

        float targetPitch = pitchForward ? tiltAngle : 0;

        ai.transform.rotation = Quaternion.Slerp(
            Quaternion.Euler(currentRotation.x, currentRotation.y, currentRotation.z),  
            Quaternion.Euler(targetPitch, currentRotation.y, currentRotation.z),         
            Time.deltaTime * rotationSpeed
        );
    }
}
