using UnityEngine;

public class ObjectTrapper : MonoBehaviour
{
    private Rigidbody trappedObject;
    private bool isTrapped = false;
    public float shootForce = 10f;
    public float kickAngle = 45f; // Angle of the kick in degrees
    public KeyCode normalKickKey = KeyCode.Space;
    public KeyCode angledKickKey = KeyCode.LeftControl;
    public KeyCode customActionKey = KeyCode.Return;
    public float forwardOffsetMagnitude = 0f; // Magnitude of the offset in the forward direction
    public float yOffset = 0f; // Offset in the Y-axis direction

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("ball"))
        {
            Rigidbody rb = other.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = true;
                rb.useGravity = false;
            }

            other.transform.SetParent(transform, true);

            trappedObject = rb;
            isTrapped = true;

            // Calculate the offset vector in the forward direction
            Vector3 forwardOffset = transform.forward * forwardOffsetMagnitude;

            // Calculate the offset vector in the Y-axis direction
            Vector3 yDirection = Vector3.up * yOffset;

            // Move the trapped object to the middle of the collider with offset
            trappedObject.transform.position = transform.position + forwardOffset + yDirection;
        }
    }

    private void Update()
    {
        // Add your movement logic here

        if (isTrapped)
        {
            if (Input.GetKey(normalKickKey))
            {
                // Normal kick
                Vector3 shootDirection = transform.forward;

                ShootBall(shootDirection);
            }
            else if (Input.GetKey(angledKickKey))
            {
                // Kick at an angle
                Vector3 localForward = transform.forward;
                Quaternion kickRotation = Quaternion.AngleAxis(-kickAngle, transform.right);
                Vector3 kickDirection = kickRotation * localForward;

                ShootBall(kickDirection);
            }
            else if (Input.GetKey(customActionKey))
            {
                // Custom action
                // Add your custom action code here
            }
        }
    }

    private void ShootBall(Vector3 direction)
    {
        trappedObject.transform.SetParent(null, true);
        trappedObject.isKinematic = false;
        trappedObject.useGravity = true;

        trappedObject.AddForce(direction * shootForce, ForceMode.Impulse);

        trappedObject = null;
        isTrapped = false;
    }
}

