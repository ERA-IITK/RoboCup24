using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS };

    public class AGVController : MonoBehaviour
    {
        public ControlMode mode = ControlMode.ROS;

        private Rigidbody robotBody;

        public float maxLinearSpeed = 2; // m/s
        public float maxRotationalSpeed = 1; // rad/s
        public float forceLimit = 10;
        public float damping = 10;

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
            MoveCube();
        }

        private void SetParameters(Rigidbody body)
        {
            body.maxAngularVelocity = maxRotationalSpeed;
            body.drag = damping;
            body.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }

        private void MoveCube()
        {
            float linearSpeed = 0f;
            float angularSpeed = 0f;

            if (mode == ControlMode.Keyboard)
            {
                linearSpeed = Input.GetAxis("Vertical") * maxLinearSpeed;
                angularSpeed = Input.GetAxis("Horizontal") * maxRotationalSpeed;
            }
            else if (mode == ControlMode.ROS)
            {
                if (Time.time - lastCmdReceived > ROSTimeout)
                {
                    rosLinear = 0f;
                    rosAngular = 0f;
                }
                linearSpeed = rosLinear;
                angularSpeed = -rosAngular;
            }

            Vector3 movement = new Vector3(0f, 0f, linearSpeed); // Move along the X-axis
            Quaternion rotation = Quaternion.Euler(0f, angularSpeed, 0f);

            robotBody.velocity = transform.TransformDirection(movement);
            robotBody.MoveRotation(robotBody.rotation * rotation);
        }
    }
}

