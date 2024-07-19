using System.Collections;
using System.Collections.Generic;
using AWSIM;
using AWSIM.TrafficSimulation;
using UnityEngine;

public class NPCScenario : MonoBehaviour
{
    public GameObject autowareEgoCar;

    // taxi, hatchback, small car, truck, van prefabs, respectively
    public GameObject npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan;

    [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
    private LayerMask vehicleLayerMask;
    [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
    private LayerMask groundLayerMask;

    // Start is called before the first frame update
    void Start()
    {
        CustomNPCSpawningManager.Initialize(this.gameObject, autowareEgoCar, npcTaxi,
            npcHatchback, npcSmallCar, npcTruck, npcVan,
            vehicleLayerMask, groundLayerMask);
        //Scenario7();
    }

    //Update is called once per frame
    void FixedUpdate()
    {
        // this should be enable even no scenario is specified
        CustomNPCSpawningManager.Manager()?.UpdateNPCs();
    }

    // a scenario
    private void Scenario1()
    {
        // set initial position on lane 239, 15m from the begining of the lane
        LanePosition spawnPosition = new LanePosition("TrafficLane.239", 15f);

        // define route, i.e., from lane 239 go straight to lane 448, and then change to lane 265
        List<string> route = new List<string>()
            {
                "TrafficLane.239",
                "TrafficLane.448",
                "TrafficLane.265"
            };
        // desired speeds, defined for each lane
        var desiredSpeeds = new Dictionary<string, float>()
            {
                { "TrafficLane.448", 20f },
                { "TrafficLane.265", 7f },
            };
        // set goal
        // stop on lane 265, 40m far from the starting point of the lane
        // Note that you can set the 2nd param to very big number (e.g., float.MaxValue),
        //  in that case, the vehicle will stop at the end of the goal lane
        var goal = new LanePosition("TrafficLane.265", 60f);

        CustomNPCSpawningManager.SpawnNPC("taxi", spawnPosition, route, desiredSpeeds, goal);
    }


    // another scenario
    private void Scenario2()
    {
        NPCVehicle npc = CustomNPCSpawningManager.PoseObstacle("small-car", "TrafficLane.263", 20f);
    }

    // another scenario
    private void Scenario3()
    {
        // set initial position
        LanePosition spawnPosition = new LanePosition("TrafficLane.239", 15f);

        // define route
        List<string> route = new List<string>()
            {
                "TrafficLane.239",
                "TrafficLane.448",
                "TrafficLane.265",
                "TrafficLane.268"
            };
        // desired speeds, defined for each lane
        var desiredSpeeds = new Dictionary<string, float>()
            {
                { "TrafficLane.448", 20f },
                { "TrafficLane.265", 7f },
            };
        // set goal
        // stop on lane 265, 40m far from the starting point of the lane
        var goal = new LanePosition("TrafficLane.268", 20f);

        CustomNPCSpawningManager.SpawnNPC("taxi", spawnPosition, route, desiredSpeeds, goal);

        // an obstacle
        //PoseObstacle("TrafficLane.263", 15f);
    }

    // spawn an NPC and make it move 5 seconds later
    private void Scenario4()
    {
        // set initial position
        LanePosition spawnPosition = new LanePosition("TrafficLane.240", 2f);

        // define route
        List<string> route = new List<string>()
            {
                "TrafficLane.240",
                "TrafficLane.422",
            };
        var desiredSpeeds = new Dictionary<string, float>();
        // set goal
        // stop on lane 265, 40m far from the starting point of the lane
        var goal = new LanePosition("TrafficLane.240", 30f);

        CustomNPCSpawningManager.SpawnNPCAndDelayMovement("taxi", spawnPosition, route, desiredSpeeds, goal, NPCSpawnDelay.Delay(5f));
    }

    // spawn an NPC with 5 seconds delay
    private void Scenario5()
    {
        // set initial position on lane 239, 15m from the begining of the lane
        LanePosition spawnPosition = new LanePosition("TrafficLane.239", 15f);

        // define route, i.e., from lane 239 go straight to lane 448, and then change to lane 265
        List<string> route = new List<string>()
            {
                "TrafficLane.239",
                "TrafficLane.448",
                "TrafficLane.265"
            };
        // desired speeds, defined for each lane
        var desiredSpeeds = new Dictionary<string, float>()
            {
                { "TrafficLane.448", 20f },
                { "TrafficLane.265", 7f },
            };
        // set goal
        // stop on lane 265, 40m far from the starting point of the lane
        var goal = new LanePosition("TrafficLane.265", 60f);

        CustomNPCSpawningManager.SpawnNPCWithDelay("taxi", spawnPosition, route, desiredSpeeds, goal, NPCSpawnDelay.Delay(5f));
    }

    // spawn an NPC and make it move at 1 second after the Ego moves
    private void Scenario6()
    {
        // set initial position
        LanePosition spawnPosition = new LanePosition("TrafficLane.240", 7f);

        // define route
        List<string> route = new List<string>()
            {
                "TrafficLane.240",
                "TrafficLane.422",
            };
        // desired speeds, defined for each lane
        var desiredSpeeds = new Dictionary<string, float>() { };
        // set goal
        // stop on lane 265, 40m far from the starting point of the lane
        var goal = new LanePosition("TrafficLane.240", 30f);

        CustomNPCSpawningManager.SpawnNPCAndDelayMovement("taxi", spawnPosition, route, desiredSpeeds, goal, NPCSpawnDelay.DelayUntilEgoMove(1f));
    }

    // spawn an NPC, delay its movement, and make it move when the Ego gets plan trajectory
    private void Scenario7()
    {
        // set initial position on lane 239, 15m from the begining of the lane
        LanePosition spawnPosition = new LanePosition("TrafficLane.239", 15f);

        // define route, i.e., from lane 239 go straight to lane 448, and then change to lane 265
        List<string> route = new List<string>()
            {
                "TrafficLane.239",
                "TrafficLane.448",
                "TrafficLane.265"
            };
        // undefined desired speeds (i.e., NPC will obey the speed limit of each lane)
        var desiredSpeeds = new Dictionary<string, float>();
        // set goal
        // stop on lane 265, 40m far from the starting point of the lane
        var goal = new LanePosition("TrafficLane.265", 60f);

        CustomNPCSpawningManager.SpawnNPCAndDelayMovement("taxi", spawnPosition, route, desiredSpeeds, goal, NPCSpawnDelay.DelayUntilEgoEngaged(0f));
    }
}
