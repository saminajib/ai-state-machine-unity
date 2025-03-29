using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform player;
    public float distance = 25f; // Distance behind the player
    public float height = 7f;   // Height above the player
    public float smoothTime = 0.3f; // Time for the camera to reach the target position
    private Vector3 velocity = Vector3.zero;

    private void LateUpdate()
    {
        if (player == null)
        {
            player = GameObject.Find("Drone").transform; // Find the drone if it's not assigned
        }

        // Get the player's rotation around only the Y-axis (yaw), ignoring pitch and roll
        Quaternion rotation = Quaternion.Euler(0, player.eulerAngles.y, 0);

        // Calculate the target position based on the player's yaw rotation, keeping a fixed height
        Vector3 targetPosition = player.position - rotation * Vector3.forward * distance + Vector3.up * height;

        // Smoothly move to the target position
        transform.position = Vector3.SmoothDamp(transform.position, targetPosition, ref velocity, smoothTime);

        // Optionally rotate the camera to always look at the player
        transform.LookAt(player);
    }
}



