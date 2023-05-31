using RosMessageTypes.PsoQ;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PSOIndividual : MonoBehaviour {
    public Vector3 pbest;
    public double pbestFitness;

    [HideInInspector]
    public float maxMovingDistance = 1.0f;

    public Vector3 lastVelocity;

    // Start is called before the first frame update
    void Start() {
        pbest = transform.position;
        pbestFitness = double.MaxValue;
    }

    // Update is called once per frame
    void Update() {

    }

    public void UpdatePbest() {
        pbest = transform.position;
    }

    public void UpdateFitness(double fitness) {
        pbestFitness = fitness;
    }

    public void UpdateLastVelocity(PosRotMsg msg) {
        lastVelocity = new Vector3(msg.pos_x, msg.pos_y, msg.pos_z);
    }

    public void ResetPBest() {
        pbest = transform.position;
        pbestFitness = double.MaxValue;
    }

    public void ResetLastVelocity() {
        lastVelocity = Vector3.zero;
    }

    public void ResetFitness() {
        pbestFitness = double.MaxValue;
    }
}
