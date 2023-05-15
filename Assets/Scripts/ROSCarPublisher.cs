using RosMessageTypes.PsoQ;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ROSCarPublisher : MonoBehaviour {
    public string topicName;
    ROSConnection ros;
    // Start is called before the first frame update
    void Start() {
        topicName = transform.gameObject.name + "/cmd_vel";
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    // Update is called once per frame
    void Update() {

    }

    public void ExecuteVelCmd(PosRotMsg msg) {
        msg.pos_x += this.transform.position.x;
        msg.pos_z += this.transform.position.z;
        ros.Publish(topicName, msg);
    }

}
