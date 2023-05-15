using RosMessageTypes.PsoQ;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ROSCarSubscriber : MonoBehaviour {
    ROSConnection ros;
    public string topicName;

    // Start is called before the first frame update
    void Start() {
        ros = ROSConnection.GetOrCreateInstance();
        topicName = transform.gameObject.name + "/cmd_vel";
        ros.Subscribe<PosRotMsg>(topicName, ExecuteVelCmd);
    }

    // Update is called once per frame
    void Update() {

    }

    void ExecuteVelCmd(PosRotMsg msg) {
        transform.gameObject.GetComponent<CarController>().SetDestination(new Vector3(msg.pos_x, msg.pos_y, msg.pos_z));
    }
}
