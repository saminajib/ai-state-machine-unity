using UnityEngine;

public class AIInputController : MonoBehaviour
{
    public GameObject aiObject; 
    public Vector3 destination = new Vector3(-30, 70f, 900f);
    private AIStateMachine aiStateMachine;

    void Start()
    {
        aiObject = GameObject.Find("Drone");
        if (aiObject != null)
        {
            aiStateMachine = aiObject.GetComponent<AIStateMachine>();
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter))
        {
            StartDrone();
        }
    }

    void StartDrone()
    {
        if (aiStateMachine == null)
        {
            return;
        }

        aiStateMachine.enabled = true; 
        aiStateMachine.SetDestination(destination);
    }
}