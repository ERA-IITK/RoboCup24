using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp_4w.Control
{
    public class SphericalWheelController : MonoBehaviour
    {
        public float wheelRadius = 0.1f; // Radius of the spherical wheel
        public float maxLinearSpeed = 2f; // Maximum linear speed in m/s
        public float maxAngularSpeed = 2f; // Maximum angular speed in rad/s

        private ArticulationBody wheelJoint;
        private float linearSpeed;
        private float angularSpeed;

        private ROSConnection ros;

        void Start()
        {
            wheelJoint = GetComponent<ArticulationBody>();
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveROSCmd);
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            linearSpeed = Mathf.Clamp((float)cmdVel.linear.x, -maxLinearSpeed, maxLinearSpeed);
            angularSpeed = Mathf.Clamp((float)cmdVel.angular.z, -maxAngularSpeed, maxAngularSpeed);
        }

        void FixedUpdate()
        {
            // Calculate the linear and angular velocities based on the input speeds
            Vector3 linearVelocity = transform.forward * linearSpeed;
            Vector3 angularVelocity = transform.up * angularSpeed;

            // Calculate the desired velocity at the point of contact with the spherical wheel
            Vector3 contactVelocity = linearVelocity + Vector3.Cross(angularVelocity, transform.position);

            // Calculate the desired rotational velocity of the wheel
            float rotationalVelocity = contactVelocity.magnitude / wheelRadius;

            // Set the joint's target velocity to simulate the spherical wheel behavior
            ArticulationDrive drive = wheelJoint.xDrive;
            drive.targetVelocity = rotationalVelocity;
            wheelJoint.xDrive = drive;
        }
    }
}
