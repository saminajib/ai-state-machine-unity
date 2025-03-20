using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform player; // Drag the player GameObject here
    public Vector3 offset = new Vector3(0, 10, -10); // Adjust the offset as needed

    private void LateUpdate()
    {
        if (player != null)
        {
            transform.position = player.position + offset;

            transform.LookAt(player);
        }
    }
}

