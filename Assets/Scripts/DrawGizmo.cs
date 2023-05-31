using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// A script that enable object to draw Gizmo in the Scene view

public class DrawGizmo : MonoBehaviour {
    public Color gizmoColor = Color.red;
    // Start is called before the first frame update
    void Start() {

    }

    // Update is called once per frame
    void Update() {

    }
    private void OnDrawGizmos() {
        Gizmos.color = gizmoColor;
        // Draw a cube at the transform position according to the size of the object
        Gizmos.DrawCube(transform.position, transform.localScale);
    }
}
