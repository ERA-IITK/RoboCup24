using UnityEngine;

public class ObjectMovement : MonoBehaviour
{
    [SerializeField] private float startX;        // Starting X coordinate
    [SerializeField] private float endX;          // Ending X coordinate
    [SerializeField] private float speed;         // Speed of movement
    [SerializeField] private float zCoordinate;   // Z coordinate

    private Vector3 startPos;
    private Vector3 endPos;
    private bool movingRight = true;

    void Start()
    {
        startPos = new Vector3(startX, transform.position.y, zCoordinate);
        endPos = new Vector3(endX, transform.position.y, zCoordinate);
    }

    void Update()
    {
        Vector3 targetPos = movingRight ? endPos : startPos;
        transform.position = Vector3.MoveTowards(transform.position, targetPos, speed * Time.deltaTime);

        if (transform.position == targetPos)
        {
            movingRight = !movingRight;
        }
    }
}

