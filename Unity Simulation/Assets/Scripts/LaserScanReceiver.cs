using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class LaserScanReceiver : MonoBehaviour
{
    ROSConnection ros;
    public string topic = "/lidar";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(topic, OnLaserScanReceived);
    }

    void OnLaserScanReceived(LaserScanMsg laserScan)
    {
        float[] ranges = laserScan.ranges;

        
        for (int i = 0; i < ranges.Length; i++)
        {
            float depth = ranges[i];
            
            Debug.Log($"Depth at index {i}: {depth}");
        }
    }
}

