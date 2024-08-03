using System.Collections;
using System.Collections.Generic;
using AWSIM;
using AWSIM.TrafficSimulation;
using UnityEngine;
using AWSIM_Script.Object;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class NPCScenario : MonoBehaviour
    {
        // Start is called before the first frame update
        void Start()
        {
            while (CustomNPCSpawningManager.Manager() == null) ;
            Scenario1();
            Test1();
        }

        //Update is called once per frame
        void FixedUpdate()
        {
        }

        private void Test1()
        {
            TrafficLane[] allLanes = CustomNPCSpawningManager.GetAllTrafficLanes();
            CustomSimUtils.LeftLaneOffset("TrafficLane.247", 15f, allLanes, out TrafficLane lane1, out float offset1);
            CustomSimUtils.RightLaneOffset("TrafficLane.247", 15f, allLanes, out TrafficLane lane12, out float offset12);
            Debug.Log("[NPCSim] lane1: " + lane1 + " at " + offset1);
            Debug.Log("[NPCSim] lane12: " + lane12 + " at " + offset12);
        }

        // a scenario
        private void Scenario1()
        {
            // set initial position on lane 239, 15m from the begining of the lane
            LaneOffsetPosition spawnPosition = new LaneOffsetPosition("TrafficLane.239", 15f);

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
            // in that case, the vehicle will stop at the end of the goal lane
            var goal = new LaneOffsetPosition("TrafficLane.265", 60f);

            CustomNPCSpawningManager.SpawnNPC(new NPCCar(
                VehicleType.TAXI, spawnPosition, goal,
                new NPCConfig(route, desiredSpeeds)));
        }

        // another scenario
        private void Scenario2()
        {
            NPCVehicle npc = CustomNPCSpawningManager.PoseObstacle(VehicleType.SMALL_CAR,
                new LaneOffsetPosition("TrafficLane.263", 20f));
        }

        // another scenario
        private void Scenario3()
        {
            // set initial position
            LaneOffsetPosition spawnPosition = new LaneOffsetPosition("TrafficLane.239", 15f);

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
            var goal = new LaneOffsetPosition("TrafficLane.268", 20f);

            CustomNPCSpawningManager.SpawnNPC(new NPCCar(
                VehicleType.TAXI, spawnPosition, goal,
                new NPCConfig(route, desiredSpeeds)));
        }

        // spawn an NPC and make it move 5 seconds later
        private void Scenario4()
        {
            // set initial position
            LaneOffsetPosition spawnPosition = new LaneOffsetPosition("TrafficLane.240", 2f);

            // define route
            List<string> route = new List<string>()
            {
                "TrafficLane.240",
                "TrafficLane.422",
            };
            var desiredSpeeds = new Dictionary<string, float>();
            // set goal
            // stop on lane 265, 40m far from the starting point of the lane
            var goal = new LaneOffsetPosition("TrafficLane.240", 30f);

            CustomNPCSpawningManager.SpawnNPC(new NPCCar(
                VehicleType.HATCHBACK, spawnPosition, goal,
                new NPCConfig(route, desiredSpeeds),
                NPCSpawnDelay.DelayMove(5f)));
        }

        // spawn an NPC with 5 seconds delay
        private void Scenario5()
        {
            // set initial position on lane 239, 15m from the begining of the lane
            LaneOffsetPosition spawnPosition = new LaneOffsetPosition("TrafficLane.239", 15f);

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
            var goal = new LaneOffsetPosition("TrafficLane.265", 60f);

            CustomNPCSpawningManager.SpawnNPC(new NPCCar(
                VehicleType.VAN, spawnPosition, goal,
                new NPCConfig(route, desiredSpeeds),
                NPCSpawnDelay.DelaySpawn(5f)));
        }

        // spawn an NPC and make it move at 1 second after the Ego moves
        private void Scenario6()
        {
            // set initial position
            LaneOffsetPosition spawnPosition = new LaneOffsetPosition("TrafficLane.240", 7f);

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
            var goal = new LaneOffsetPosition("TrafficLane.240", 30f);

            CustomNPCSpawningManager.SpawnNPC(new NPCCar(
                VehicleType.TAXI, spawnPosition, goal,
                new NPCConfig(route, desiredSpeeds),
                NPCSpawnDelay.DelayMoveUntilEgoMove(1f)));
        }

        // spawn an NPC, delay its movement, and make it move when the Ego engaged
        private void Scenario7()
        {
            // set initial position on lane 239, 15m from the begining of the lane
            LaneOffsetPosition spawnPosition = new LaneOffsetPosition("TrafficLane.239", 15f);

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
            var goal = new LaneOffsetPosition("TrafficLane.265", 60f);

            CustomNPCSpawningManager.SpawnNPC(new NPCCar(
                VehicleType.TRUCK, spawnPosition, goal,
                new NPCConfig(route, desiredSpeeds),
                NPCSpawnDelay.DelayMoveUntilEgoEngaged(0f)));
        }
    }

}