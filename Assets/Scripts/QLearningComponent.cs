using RosMessageTypes.PsoQ;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class QLearningComponent : MonoBehaviour {
    public Tuple<int, int> currentState;
    public float alpha = 0.1f;
    public float epsilon = 0.1f;
    public float gamma = 0.9f;

    //[HideInInspector]
    public int actionSize;
    //[HideInInspector]
    public float angleStepSize;
    //[HideInInspector]
    public float distanceStepSize;

    public float maxMovingDistance;
    public float maxDistanceThreshold = 5.0f;

    private float[][][] qTable; //QTable [angle][distance][action]
    private int[] actionSpace;
    // Start is called before the first frame update
    void Start() {
        int distanceSize = (int)(maxDistanceThreshold / distanceStepSize) + 1;
        int angleSize = (int)(360 / angleStepSize);
        qTable = new float[angleSize][][];
        for (int i = 0; i < angleSize; i++) {
            qTable[i] = new float[distanceSize][];
            for (int j = 0; j < distanceSize; j++) {
                qTable[i][j] = new float[actionSize + 1];
            }
        }
        actionSpace = new int[actionSize + 1];
        for (int i = 0; i < actionSize + 1; i++) {
            actionSpace[i] = i;
        }
        Debug.Log("Size of qTable: " + qTable.Count().ToString() + " Size of qT[0]: " + qTable[0].Count().ToString() + " Size of qT[0][0]: " +
            qTable[0][0].Count().ToString());
    }

    // Update is called once per frame
    void Update() {

    }

    public float[] GetQValue(int angleBin, int distanceBin) {
        return qTable[angleBin][distanceBin];
    }

    public int ChooseAction(int angleBin, int distanceBin) {
        float[] qValues = GetQValue(angleBin, distanceBin);
        int action = 0;
        if (UnityEngine.Random.Range(0.0f, 1.0f) < epsilon) {
            action = UnityEngine.Random.Range(0, actionSize + 1);
        } else {
            action = Array.IndexOf(qValues, Mathf.Max(qValues));
        }
        return action;
    }

    public void TakeAction(int action) {
        PosRotMsg msg = new PosRotMsg();
        if (action == 0) {
            msg.pos_x = 0;
            msg.pos_z = 0;
        } else {
            float angle = (action - 1) * angleStepSize * Mathf.PI / 180;
            msg.pos_x = maxMovingDistance * Mathf.Cos(angle);
            msg.pos_z = maxMovingDistance * Mathf.Sin(angle);
        }
        this.gameObject.GetComponent<ROSCarPublisher>().ExecuteVelCmd(msg);
    }

    public void UpdateQTable(int angleBin, int distanceBin, int action, float reward, int newAngleBin, int newDistanceBin) {
        Debug.Log("Update QTable: " + angleBin + " " + distanceBin + " " + action + " " + reward + " " + newAngleBin + " " + newDistanceBin);
        qTable[angleBin][distanceBin][action] = (1 - alpha) * qTable[angleBin][distanceBin][action] + alpha * (reward + gamma * Mathf.Max(qTable[newAngleBin][newDistanceBin]));
    }
}
