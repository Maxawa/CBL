using System.Collections;
using System.Collections.Generic;
using ROS2.Interfaces;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

//Part of this code is adapted from OccupancyGridDefaultVisualizer.cs
public class MapAlgorithm : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string topic = "/map";
       // ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>(topic, AddMessage);
    }

    public void AddMessage(Message message)
    {
      //  message = (OccupancyGridMsg)message;
    }
    // Update is called once per frame
    void Update()
    {

    }
}
