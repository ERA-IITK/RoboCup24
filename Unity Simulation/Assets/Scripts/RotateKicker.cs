using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public enum RotateKickerControlMode { Keyboard, ROS };

    public class RotateKicker : MonoBehaviour
    {
        public RotateKickerControlMode mode = RotateKickerControlMode.ROS;

        private Rigidbody robotBody;

        public float maxLinearSpeed = 2; // m/s
        public float maxRotationalSpeed = 1; // rad/s
        public float forceLimit = 10;
        public float damping = 10;
        public GameObject pivotObject;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            robotBody = GetComponent<Rigidbody>();
            SetParameters(robotBody);

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveROSCmd);
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            RotateCube();
        }

        private void SetParameters(Rigidbody body)
        {
            body.maxAngularVelocity = maxRotationalSpeed;
            body.drag = damping;
            body.constraints = RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;
        }

        private void RotateCube()
        {
            float linearSpeed = 0f;
            float rotationSpeed = 0f;

            if (mode == RotateKickerControlMode.Keyboard)
            {
                linearSpeed = Input.GetAxis("Vertical") * maxLinearSpeed;
                rotationSpeed = Input.GetAxis("Horizontal") * maxRotationalSpeed;
            }
            else if (mode == RotateKickerControlMode.ROS)
            {
                if (Time.time - lastCmdReceived > ROSTimeout)
                {
                    rosLinear = 0f;
                    rosAngular = 0f;
                }
                linearSpeed = rosLinear;
                rotationSpeed = -rosAngular;
            }

            Vector3 rotationAxis = new Vector3(0f, 0f, linearSpeed); // Set rotation axis
            float rotationAngle = rotationSpeed * Time.deltaTime; // Set rotation angle

            transform.RotateAround(pivotObject.transform.position, rotationAxis, rotationAngle);
        }
    }
}

