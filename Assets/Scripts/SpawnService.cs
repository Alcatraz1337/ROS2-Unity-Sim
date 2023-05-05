using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.PsoQ;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class SpawnService : MonoBehaviour
{
    [SerializeField]
    string m_ServiceName = "Spawn";
    public GameObject objectToSpawn;
    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().ImplementService<SpawnRequest, SpawnResponse>(m_ServiceName, HandleSpawnRequest);
    }

    SpawnResponse HandleSpawnRequest(SpawnRequest request) {
    
        Vector3 spawnVector = new Vector3(request.x, request.y, request.z);
        string name = request.name;
        // Spawn a cube
        GameObject botToSpawn = Instantiate(objectToSpawn, spawnVector, Quaternion.identity);
        botToSpawn.name = name;
        return new SpawnResponse { name = name };
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
