using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp_4w.Control
{
    public enum ControlMode { Keyboard, ROS }

    public class AGVController_4w : MonoBehaviour
    {
        public GameObject frontLeftWheel;
        public GameObject frontRightWheel;
        public GameObject rearLeftWheel;
        public GameObject rearRightWheel;
        public ControlMode mode = ControlMode.ROS;

        private ArticulationBody frontLeftWheelAB;
        private ArticulationBody frontRightWheelAB;
        private ArticulationBody rearLeftWheelAB;
        private ArticulationBody rearRightWheelAB;

        public float maxLinearSpeed = 2; // m/s
        public float maxRotationalSpeed = 1; // rad/s
        public float wheelRadius = 0.033f; // meters
        public float trackWidth = 0.288f; // meters Distance between tires
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        private ROSConnection ros;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        private void Start()
        {
            frontLeftWheelAB = frontLeftWheel.GetComponent<ArticulationBody>();
            frontRightWheelAB = frontRightWheel.GetComponent<ArticulationBody>();
            rearLeftWheelAB = rearLeftWheel.GetComponent<ArticulationBody>();
            rearRightWheelAB = rearRightWheel.GetComponent<ArticulationBody>();
            SetParameters(frontLeftWheelAB);
            SetParameters(frontRightWheelAB);
            SetParameters(rearLeftWheelAB);
            SetParameters(rearRightWheelAB);

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveROSCmd);
        }

        private void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        private void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyboardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void KeyboardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            float inputRotationSpeed;
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            float turnDirection = Input.GetAxis("Horizontal");
            if (turnDirection > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirection < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            RobotInput(inputSpeed, inputRotationSpeed);
        }

        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinear, -rosAngular);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }

            float wheel1Rotation = (speed / wheelRadius) * Mathf.Rad2Deg;
            float wheel2Rotation = wheel1Rotation;
            float wheel3Rotation = (speed / wheelRadius) * Mathf.Rad2Deg;
            float wheel4Rotation = wheel3Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius) * Mathf.Rad2Deg;

            wheel1Rotation += wheelSpeedDiff / 2;
            wheel2Rotation += wheelSpeedDiff / 2;
            wheel3Rotation -= wheelSpeedDiff / 2;
            wheel4Rotation -= wheelSpeedDiff / 2;

            SetSpeed(frontLeftWheelAB, wheel1Rotation);
            SetSpeed(frontRightWheelAB, wheel2Rotation);
            SetSpeed(rearLeftWheelAB, wheel3Rotation);
            SetSpeed(rearRightWheelAB, wheel4Rotation);
        }
    }
}
