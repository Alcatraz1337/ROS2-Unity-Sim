using RosMessageTypes.PsoQ;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics;
using UnityEngine;

public static class Parameters {
    [Serializable]
    public class ROSParameters {
        [Header("ROS Parameters")]
        public string serviceName = "Spawn";
        public string topicName = "pos_topic";
        public string rosBridgeIP = "";
        public int rate = 10;
    }

    [Serializable]
    public class EnvParameters {
        public float spawnBoundary = 4.5f;

    }

    [Serializable]
    public class PsoParameters {
        [Header("PSO Parameters")]
        public int population = 5;
        [HideInInspector]
        public int steps = 0;
        public int maxSteps = 10;
        public PoseWithFitnessMsg gbestPose;
        public int n_escapers = 1;
        public float inertiaWeight = 0.5f;
        public float c1 = 2.0f;
        public float c2 = 2.0f;
    }

    [Serializable]
    public class QLearningParameters {
        [Header("Q-Learning Parameters")]
        [HideInInspector]
        public int epoch = 0;
        public int maxEpoch = 10;
        public float targetDetectRadius = 5.0f;
        public float targetCaughtRadius = 0.2f;
        public bool isTargetCaught = false;
        public float angleStepSize = 45.0f;
        public float distanceStepSize = 1.0f;
    }
}