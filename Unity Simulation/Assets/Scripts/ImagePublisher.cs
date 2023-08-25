using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class ImagePublisher : MonoBehaviour
{
    public Camera imageCamera;


    ROSConnection ros;
    public string rgbTopicName = "/camera1/rgb";


    public float publishMessageFrequency = 0.05f;
    private float timeElapsed;

    private RenderTexture renderTexture;

    private Texture2D texture; // Reused texture object


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(rgbTopicName);


        int width = Screen.width;
        int height = Screen.height;
        renderTexture = new RenderTexture(width, height, 24);

        imageCamera.targetTexture = renderTexture;

        texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);

    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            RenderTexture.active = renderTexture;
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);
            texture.Apply();


            Color32[] rgbPixels = texture.GetPixels32();
            byte[] rgbBytes = new byte[rgbPixels.Length * 4];

            for (int i = 0; i < rgbPixels.Length; i++)
            {
                rgbBytes[i * 4] = rgbPixels[i].r;
                rgbBytes[i * 4 + 1] = rgbPixels[i].g;
                rgbBytes[i * 4 + 2] = rgbPixels[i].b;
                rgbBytes[i * 4 + 3] = rgbPixels[i].a;
            }


            ImageMsg rgbImageMsg = new ImageMsg
            {
                header = new HeaderMsg(),
                height = (uint)renderTexture.height,
                width = (uint)renderTexture.width,
                encoding = "rgba8",
                step = (uint)(renderTexture.width * 4),
                data = rgbBytes
            };



            ros.Publish(rgbTopicName, rgbImageMsg);


            timeElapsed = 0;
        }
    }
}

