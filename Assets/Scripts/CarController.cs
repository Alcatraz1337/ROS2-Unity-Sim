using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour {
    public float speed = 10.0f;
    public float rotationSpeed = 100.0f;
    public float maxSpeed = 10.0f;
    public Vector3 destination;
    public float delta = 0.05f;
    public bool isMoving = false;

    // Start is called before the first frame update
    void Start() {
        destination = transform.position;
    }

    // Update is called once per frame
    void Update() {
        if (Vector3.Distance(transform.position, destination) > delta) {
            isMoving = true;
            MoveToPosition(destination);
        } else {
            isMoving = false;
        }
    }

    private void MoveToPosition(Vector3 destination) {
        float step = speed * Time.deltaTime; // calculate distance to move
        transform.position = Vector3.MoveTowards(transform.position, destination, step);
    }

    public void SetDestination(Vector3 destination) {
        this.destination = destination;
    }
}
