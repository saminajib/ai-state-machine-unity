using UnityEngine;
using System.Collections.Generic;

//parallel state machine logic
public class AIStateMachine : MonoBehaviour
{
    public Vector3 destination = new Vector3(-30, 70f, 10f);
    public float height = 20f;
    public bool SidewaysProximityStateOn = true;
    private IAIState currentState;
    private List<IAIState> activeStates = new List<IAIState>();

    private void OnEnable()
    {
        AddState(new TakeoffState());
    }

    private void Update()
    {
        var statesToExecute = new List<IAIState>(activeStates); 
        foreach (var state in statesToExecute)
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

    public void SetDestination(Vector3 newDestination)
    {
        destination = newDestination;
    }

    public void OnDisable()
    {
        activeStates = new List<IAIState>();
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

    public void Execute(AIStateMachine ai)
    {
        //dont want destination to be affected by height
        ai.destination.y = ai.transform.position.y;

        Vector3 bestDirection = FindBestPath(ai);

        //find a path forward else go up
        if(bestDirection != Vector3.up)
        {
            fovAngle = 170f;
            ai.SidewaysProximityStateOn = true;

            AIHelper.ControlYaw(ai, bestDirection);
            AIHelper.ControlPitch(ai, true);
            AIHelper.MoveForwardBasedOnPitch(ai);

        }
        else
        {
            //no roll control needed and decrease angle of view
            fovAngle = 50f;
            ai.SidewaysProximityStateOn = false;

            AIHelper.ControlYaw(ai, ai.destination);
            AIHelper.ControlPitch(ai, false);
            
            ai.transform.position += ai.transform.up * moveSpeed * Time.deltaTime; 
        }
    }

    Vector3 FindBestPath(AIStateMachine ai)
    { 
        //default to up if no other options
        Vector3 bestDirection = Vector3.up;             

        for(int i = 0; i < 3; i++)
        {
            if(bestDirection == Vector3.up)
            {
                float rayDistance = 70f - 20f * i;
                bestDirection = FindBestPathRaycastShooter(ai, rayDistance);
            }
        }

        return bestDirection;
    }

    Vector3 FindBestPathRaycastShooter(AIStateMachine ai, float rayDistance)
    {
        Vector3 bestDirection = Vector3.up;

        //aim directly towards target      
        Vector3 directionToTarget = AIHelper.GetDirectionToTarget(ai.transform.position, ai.destination);
        
        float angleStep = fovAngle / rayCount;

        for (int i = 0; i < rayCount; i++)
        {
            //fan out from center rather than prioritizing one direction first
            float angle = (i % 2 == 0 ? 1 : -1) * (i / 2) * angleStep;

            Vector3 direction = Quaternion.Euler(0, angle, 0) * directionToTarget;

            //slightly move ray cast up might help with bugs
            Vector3 origin = ai.transform.position + Vector3.up * 0.5f + Vector3.forward * 0f; 

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

    public float forwardAngle = 75f;

    public void Execute(AIStateMachine ai)
    {
        if(!ai.SidewaysProximityStateOn)
            return;
        
        PitchLeftOrRight(ai);
        AIHelper.MoveSidewaysBasedOnRoll(ai);
    }

    void PitchLeftOrRight(AIStateMachine ai)
    {
        RaycastHit leftForwardHit, rightForwardHit, leftHit, rightHit;
        Vector3 rightOrigin = ai.transform.position + Vector3.up * 0.5f + Vector3.forward * 3f + Vector3.right * 1.5f; 
        Vector3 leftOrigin = ai.transform.position + Vector3.up * 0.5f + Vector3.forward * 3f + Vector3.left * 1.5f; 


        Vector3 leftDirection = Vector3.Cross(ai.transform.forward, Vector3.up).normalized;
        Vector3 rightDirection = Vector3.Cross(Vector3.up, ai.transform.forward).normalized;

        Vector3 forwardLeftDirection = Quaternion.Euler(0, forwardAngle, 0) * leftDirection;
        Vector3 forwardRightDirection = Quaternion.Euler(0, -1f * forwardAngle, 0) * rightDirection;

        float sidewaysRayDistance = 20f;

        Physics.SphereCast(leftOrigin, spherecastWidth, leftDirection, out leftHit, sidewaysRayDistance);
        Physics.SphereCast(rightOrigin, spherecastWidth, rightDirection, out rightHit, sidewaysRayDistance);
        
        if(leftHit.distance != 0 || rightHit.distance != 0)
        {
            Debug.DrawRay(leftOrigin, leftDirection * leftHit.distance, Color.cyan, .05f);
            Debug.DrawRay(rightOrigin, rightDirection * rightHit.distance, Color.cyan, .05f);

            AIHelper.ControlRoll(ai, true, true);
            return;
        }

        Physics.SphereCast(leftOrigin, spherecastWidth, forwardLeftDirection, out leftForwardHit, rayDistance);
        Physics.SphereCast(rightOrigin, spherecastWidth, forwardRightDirection, out rightForwardHit, rayDistance);

        float leftDistance = leftForwardHit.distance;
        float rightDistance = rightForwardHit.distance;

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
        else       //both hit
        {
            Debug.DrawRay(leftOrigin, forwardLeftDirection * leftDistance, Color.cyan, .05f);
            Debug.DrawRay(rightOrigin, forwardRightDirection * rightDistance, Color.cyan, .05f);
            
            AIHelper.ControlRoll(ai, leftDistance > rightDistance ? true : false, false);
        }
    }
}

public class TakeoffState: IAIState
{
    public void Execute(AIStateMachine ai)
    {
        RaycastHit hit;
        Vector3 origin = ai.transform.position + Vector3.up * 0.5f;
        Vector3 direction = Vector3.down;

        if (Physics.Raycast(origin, direction, out hit, ai.height))
        {
            Debug.DrawRay(origin, direction * hit.distance, Color.red, .05f);
            AIHelper.MoveUpward(ai, true, hit.distance);
        }
        else
        {
            ai.AddState(new BestPathForwardState());
            ai.AddState(new SidewaysProximityState());
            ai.RemoveState(this);
        }
    }
}


public static class AIHelper
{
    public static float moveForwardSpeed = 10f;
    public static float moveSidewaysSpeed = 5f;
    public static float moveUpSpeed = 3f;
    public static float pitchAngle = 8f;
    public static float rollAngle = 13f; 

    public static Vector3 GetDirectionToTarget(Vector3 position, Vector3 destination)
    {
        return (destination - position).normalized;
    }

    public static void MoveUpward(AIStateMachine ai, bool goUp, float distance)
    {
        float adjustedSpeed = Mathf.Sin(Mathf.PI * distance / ai.height) * moveUpSpeed * 1.5f + moveUpSpeed / 2;

        if(goUp)
            ai.transform.position += ai.transform.up * adjustedSpeed * Time.deltaTime; 
        else
            ai.transform.position -= ai.transform.up * adjustedSpeed * Time.deltaTime; 
    }

    public static void MoveForwardBasedOnPitch(AIStateMachine ai)
    {
        float xRotation = ai.transform.rotation.eulerAngles.x;

        float speedMultiplier = Mathf.Abs(xRotation) / pitchAngle;

        float adjustedSpeed = moveForwardSpeed * speedMultiplier * speedMultiplier;

        Vector3 flatForward = new Vector3(ai.transform.forward.x, 0, ai.transform.forward.z).normalized;

        ai.transform.position += flatForward * adjustedSpeed * Time.deltaTime;
    }

    public static void MoveSidewaysBasedOnRoll(AIStateMachine ai)
    {
        Vector3 direction = new Vector3(0, 0, 0);

        float zRotation = ai.transform.rotation.eulerAngles.z;

        if(zRotation > 0)
            direction = Vector3.Cross(ai.transform.forward, Vector3.up).normalized;

        if (zRotation > 180)
        {
            direction = Vector3.Cross(Vector3.up, ai.transform.forward).normalized;
            zRotation -= 360; 
        }

        float speedMultiplier = Mathf.Abs(zRotation) / 13f;

        float adjustedSpeed = moveSidewaysSpeed * speedMultiplier * speedMultiplier;

        ai.transform.position += direction * adjustedSpeed * Time.deltaTime;
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

        Quaternion baseRotation = Quaternion.LookRotation(ai.transform.forward, Vector3.up);

        float angle = rollRight ? rollAngle : -rollAngle;

        if(cancel)
            angle = 0f;

        Quaternion targetRotation = Quaternion.AngleAxis(angle, ai.transform.forward) * baseRotation;

        ai.transform.rotation = Quaternion.Slerp(ai.transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);
    }

    public static void ControlPitch(AIStateMachine ai, bool pitchForward)
    {
        float rotationSpeed = 1f;

        Vector3 currentRotation = ai.transform.rotation.eulerAngles;

        float targetPitch = pitchForward ? pitchAngle : 0;

        ai.transform.rotation = Quaternion.Slerp(
            Quaternion.Euler(currentRotation.x, currentRotation.y, currentRotation.z),  
            Quaternion.Euler(targetPitch, currentRotation.y, currentRotation.z),         
            Time.deltaTime * rotationSpeed
        );
    }
}
