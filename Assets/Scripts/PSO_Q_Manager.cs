using RosMessageTypes.PsoQ;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting;
using UnityEngine;
using static Parameters;
using Random = UnityEngine.Random;

public class PSO_Q_Manager : MonoBehaviour {
    public ROSParameters rosParameters;
    public EnvParameters envParameters;
    public PsoParameters psoParameters;
    public CarParameters carParameters;
    public QLearningParameters qLearningParameters;

    public GameObject catcherPrefab;
    public GameObject escaperPrefab;

    private List<GameObject> catchers;
    private List<GameObject> escapers;
    private bool isAllCatchersSet;
    private bool isAllEscapersSet;

    ROSConnection ros;

    // Start is called before the first frame update
    void Start() {
        ros = ROSConnection.GetOrCreateInstance();
        catchers = new List<GameObject>();
        escapers = new List<GameObject>();
        for (int i = 0; i < psoParameters.population; i++) {
            // Spawn a catcher
            GameObject catcher = Instantiate(catcherPrefab, new Vector3(Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary),
                0,
                Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary)), Quaternion.identity);
            catcher.name = "catcher" + i.ToString();
            catcher.GetComponent<PSOIndividual>().maxMovingDistance = carParameters.maxMovingDistance;
            catchers.Add(catcher);
        }
        for (int i = 0; i < psoParameters.n_escapers; i++) {
            // Spawn an escaper
            GameObject escaper = Instantiate(escaperPrefab, new Vector3(Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary),
                                            0,
                                            Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary)), Quaternion.identity);
            escaper.name = "escaper" + i.ToString();
            GameObject body = escaper.transform.GetChild(0).gameObject;
            body.GetComponent<Renderer>().material.color = Color.cyan;
            escaper.GetComponent<QLearningComponent>().angleStepSize = qLearningParameters.angleStepSize;
            escaper.GetComponent<QLearningComponent>().distanceStepSize = qLearningParameters.distanceStepSize;
            escaper.GetComponent<QLearningComponent>().actionSize = qLearningParameters.actionSize;
            escaper.GetComponent<QLearningComponent>().maxMovingDistance = carParameters.maxMovingDistance;
            escapers.Add(escaper);
        }
        isAllCatchersSet = true;
        isAllEscapersSet = true;
    }

    void FixedUpdate() {
        if (qLearningParameters.epoch < qLearningParameters.maxEpoch) {
            if (psoParameters.steps == 0 && isAllCatchersSet && isAllEscapersSet) {
                for (int i = 0; i < psoParameters.population; i++) {
                    PosRotMsg randomVel = GenerateRandomVelocity(i);
                    catchers[i].GetComponent<PSOIndividual>().UpdateLastVelocity(randomVel);
                    catchers[i].GetComponent<ROSCarPublisher>().ExecuteVelCmd(randomVel);
                }
                WaitCatchers();
                escapers[0].GetComponent<QLearningComponent>().currentState = ObserveCurrentState();
                int action = escapers[0].GetComponent<QLearningComponent>().ChooseAction(escapers[0].GetComponent<QLearningComponent>().currentState.Item1,
                    escapers[0].GetComponent<QLearningComponent>().currentState.Item2);
                escapers[0].GetComponent<QLearningComponent>().TakeAction(action);
                WaitEscapers();
                if (!escapers[0].GetComponent<CarController>().isMoving) {
                    Tuple<int, int> newState = ObserveCurrentState();
                    float reward = GenerateReward();
                }
            }
            psoParameters.gbestPose = new PoseWithFitnessMsg();
            psoParameters.gbestPose.fitness = float.MaxValue;
            UpdatePbest();
            UpdateGbest();
            // Update qLearningParameters
            qLearningParameters.epoch++;
            qLearningParameters.isTargetCaught = false;
            //UpdateIsTargetCaught();
            // Update cars
            //UpdateCars();
        }
    }

    #region PSO

    private double CalculateFitness(Transform catcher, Transform escaper) {
        return Vector3.Distance(catcher.position, escaper.position);
    }

    private void UpdatePbest() {
        for (int i = 0; i < psoParameters.population; i++) {
            // Update pbest
            double fitness = CalculateFitness(catchers[i].transform, escapers[0].transform);
            if (fitness < catchers[i].GetComponent<PSOIndividual>().pbestFitness) {
                catchers[i].GetComponent<PSOIndividual>().UpdatePbest();
                catchers[i].GetComponent<PSOIndividual>().UpdateFitness(fitness);
            }
        }
    }

    private void UpdateGbest() {
        for (int i = 0; i < psoParameters.population; i++) {
            if (catchers[i].GetComponent<PSOIndividual>().pbestFitness < psoParameters.gbestPose.fitness) {
                psoParameters.gbestPose.pos_x = catchers[i].transform.position.x;
                psoParameters.gbestPose.pos_y = catchers[i].transform.position.y;
                psoParameters.gbestPose.pos_z = catchers[i].transform.position.z;
                psoParameters.gbestPose.fitness = (float)catchers[i].GetComponent<PSOIndividual>().pbestFitness;
            }
        }
    }

    private PosRotMsg GenerateRandomVelocity(int index) {
        float randamDistance = Random.Range(0.0f, catchers[index].GetComponent<PSOIndividual>().maxMovingDistance);
        float randomAngle = Random.Range(0.0f, 2 * Mathf.PI);
        PosRotMsg randomPos = new PosRotMsg();
        randomPos.pos_x = randamDistance * Mathf.Cos(randomAngle);
        randomPos.pos_z = randamDistance * Mathf.Sin(randomAngle);

        return randomPos;
    }

    private PosRotMsg GenerateAdjustedVelocity(int index) {
        PosRotMsg adjustedVelocity = new PosRotMsg();
        float dxGbest = psoParameters.gbestPose.pos_x - catchers[index].transform.position.x;
        float dzGbest = psoParameters.gbestPose.pos_z - catchers[index].transform.position.z;

        float dxPbest = catchers[index].GetComponent<PSOIndividual>().pbest.x - catchers[index].transform.position.x;
        float dzPbest = catchers[index].GetComponent<PSOIndividual>().pbest.z - catchers[index].transform.position.z;

        float randC1 = Random.Range(0.0f, 1.0f);
        float randC2 = Random.Range(0.0f, 1.0f);

        float dxNew = psoParameters.inertiaWeight * catchers[index].GetComponent<PSOIndividual>().lastVelocity.x +
            psoParameters.c1 * randC1 * dxPbest + psoParameters.c2 * randC2 * dxGbest;
        float dzNew = psoParameters.inertiaWeight * catchers[index].GetComponent<PSOIndividual>().lastVelocity.y +
            psoParameters.c1 * randC1 * dzPbest + psoParameters.c2 * randC2 * dzGbest;

        float newDistance = Mathf.Sqrt(dxNew * dxNew + dzNew * dzNew);
        if (newDistance < catchers[index].GetComponent<PSOIndividual>().maxMovingDistance) {
            adjustedVelocity.pos_x = dxNew;
            adjustedVelocity.pos_z = dzNew;
            return adjustedVelocity;
        } else {
            float newAngle = Mathf.Atan2(dzNew, dxNew);
            adjustedVelocity.pos_x = catchers[index].GetComponent<PSOIndividual>().maxMovingDistance * Mathf.Cos(newAngle);
            adjustedVelocity.pos_z = catchers[index].GetComponent<PSOIndividual>().maxMovingDistance * Mathf.Sin(newAngle);
            return adjustedVelocity;
        }
    }
    #endregion PSO

    #region Q-Learning

    private float AngleToDegree(float radAngle) {
        return radAngle * 180 / Mathf.PI;
    }

    private int GetDistanceBin(float distance) {
        if (distance > qLearningParameters.targetDetectRadius) {
            return (int)(qLearningParameters.targetDetectRadius / qLearningParameters.distanceStepSize);
        } else {
            return (int)(distance / qLearningParameters.distanceStepSize);
        }
    }
    private int GetAngleBin(float angle) {
        return (int)((AngleToDegree(angle) + 180) / qLearningParameters.angleStepSize);
    }

    // Observe current state of the environment
    // Return: (distanceBin, angleBin)
    private Tuple<int, int> ObserveCurrentState() {
        float dx = 0.0f, dy = 0.0f;
        for (int i = 0; i < psoParameters.population; i++) {
            dx += escapers[0].transform.position.x - catchers[i].transform.position.x;
            dy += escapers[0].transform.position.z - catchers[i].transform.position.z;
        }
        float distance = Mathf.Sqrt(dx * dx + dy * dy);
        float angle = Mathf.Atan2(dy, dx);
        return new Tuple<int, int>(GetDistanceBin(distance), GetAngleBin(angle));
    }

    private float GenerateReward() {
        float dx = 0.0f, dy = 0.0f;
        for (int i = 0; i < psoParameters.population; i++) {
            dx += escapers[0].transform.position.x - catchers[i].transform.position.x;
            dy += escapers[0].transform.position.z - catchers[i].transform.position.z;
        }
        float distance = Mathf.Sqrt(dx * dx + dy * dy);
        if (distance > qLearningParameters.targetDetectRadius) {
            return 1.0f;
        } else if (distance < qLearningParameters.targetCaughtRadius) {
            return -1.0f;
        } else {
            return (distance - qLearningParameters.targetCaughtRadius) / (qLearningParameters.targetDetectRadius - qLearningParameters.targetCaughtRadius);
        }
    }

    #endregion Q-Learning

    #region Environment

    private void ResetEnvironment() {

        for (int i = 0; i < psoParameters.population; i++) {
            // Reset catcher
            catchers[i].transform.position = new Vector3(Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary),
                                                         0,
                                                         Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary));
            catchers[i].GetComponent<PSOIndividual>().ResetPBest();
            catchers[i].GetComponent<PSOIndividual>().ResetLastVelocity();
            catchers[i].GetComponent<PSOIndividual>().ResetFitness();
            // TODO: Reset escaper
            escapers[i].transform.position = new Vector3(Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary),
                                                         0,
                                                         Random.Range(-envParameters.spawnBoundary, envParameters.spawnBoundary));
        }
    }

    private bool CheckTargetCaught() {
        float dx = 0.0f, dy = 0.0f;
        for (int i = 0; i < psoParameters.population; i++) {
            dx += escapers[i].transform.position.x - catchers[i].transform.position.x;
            dy += escapers[i].transform.position.z - catchers[i].transform.position.z;
            float distance = Mathf.Sqrt(dx * dx + dy * dy);
            if (distance < qLearningParameters.targetCaughtRadius)
                return true;
        }
        return false;
    }

    private bool IsAllCatchersSet() {
        for (int i = 0; i < psoParameters.population; i++) {
            if (catchers[i].GetComponent<CarController>().isMoving)
                return false;
        }
        return true;
    }

    private void WaitCatchers() {
        for (int i = 0; i < psoParameters.population; i++) {
            if (catchers[i].GetComponent<CarController>().isMoving) {
                isAllCatchersSet = false;
                return;
            }
        }
        isAllCatchersSet = true;
    }

    private void WaitEscapers() {
        for (int i = 0; i < psoParameters.n_escapers; i++) {
            if (escapers[0].GetComponent<CarController>().isMoving) {
                isAllEscapersSet = false;
                return;
            }
        }
        isAllEscapersSet = true;
    }

    #endregion Environment
}
