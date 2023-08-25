using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;

public class ObjectPositionPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/object_position";
    public GameObject targetObject;

    public float publishMessageFrequency = 1.0f;
    private float timeElapsed;

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
         
            Vector3 position = targetObject.transform.position;

           
            PoseMsg poseMsg = new PoseMsg
            {
                position = new PointMsg { x = position.x, y = position.y, z = position.z },
                orientation = new QuaternionMsg()
            };

            // Publish the pose message
            ros.Publish(topicName, poseMsg);

            timeElapsed = 0;
        }
    }
}

