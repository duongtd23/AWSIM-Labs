using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public class NPCSpawning : MonoBehaviour
    {
        public GameObject autowareEgoCar;
        private Rigidbody egoRigidbody;

        // taxi, hatchback, small car, truck, van prefabs, respectively
        public GameObject npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan ;

        private NPCVehicleSimulator npcVehicleSimulator;
        private NPCVehicleSpawner npcVehicleSpawner;
        [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        private LayerMask vehicleLayerMask;
        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        private LayerMask groundLayerMask;

        // this time starts ticking when Ego vehicle start moving
        private float autowareEgoTimer = 0;

        private Dictionary<NPCVehicle, Tuple<NPCSpawnDelay, List<string>, int, Dictionary<string, float>, LanePosition>> delayingMoveNPCs;
        private List<DelayingNPCVehicle> delayingSpawnNPCs;

        // Start is called before the first frame update
        void Start()
        {
            autowareEgoTimer = 0;
            if (autowareEgoCar == null)
                throw new UnityException("[NPCSim] Cannot detect the ego vehicle");

            egoRigidbody = autowareEgoCar.GetComponent<Rigidbody>();
            if (egoRigidbody == null)
                throw new UnityException("[NPCSim] Cannot find rigidbody of the ego vehicle");

            InitialSetup();
            Scenario5();
        }

        // initial setup
        private void InitialSetup()
        {
            // only single NPCVehicleConfig is supported for all NPCs
            // TODO: support different NPCVehicleConfig for different NPCs
            NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();
            vehicleConfig.Acceleration = 7f;
            vehicleConfig.Deceleration = 8f;
            npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, 10, autowareEgoCar);
            npcVehicleSpawner = new NPCVehicleSpawner(this.gameObject,
                new GameObject[] { npcTaxi }, new TrafficLane[] { });

            delayingMoveNPCs = new Dictionary<NPCVehicle, Tuple<NPCSpawnDelay, List<string>, int, Dictionary<string, float>, LanePosition>>();
            delayingSpawnNPCs = new List<DelayingNPCVehicle>();
        }

        //Update is called once per frame
        void FixedUpdate()
        {
            // tick the autoware timer if it already started
            if (autowareEgoTimer > 0)
                autowareEgoTimer += Time.fixedDeltaTime;
            else
                // otherwise, check if Ego moved to start the timer
                // TODO: use proper implementation instead of egoRigidbody.velocity
                if (NPCSimUtils.DistanceIgnoreYAxis(egoRigidbody.velocity, Vector3.zero) > 0.2f)
                    autowareEgoTimer += Time.fixedDeltaTime;

            npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);
            UpdateDelayingNPCs();
        }

        private void UpdateDelayingNPCs()
        {
            List<NPCVehicle> removeAfter = new List<NPCVehicle>();
            foreach (var entry in delayingMoveNPCs)
            {
                NPCVehicle npc = entry.Key;
                NPCSpawnDelay delay = entry.Value.Item1;
                
                if (delay.UntilEgoMove && autowareEgoTimer >= delay.DelayAmount)
                {
                    List<string> route = entry.Value.Item2;
                    var routeLanes = parseLanes(route);
                    npcVehicleSimulator.Register(npc, routeLanes, entry.Value.Item3, entry.Value.Item4, entry.Value.Item5);
                    removeAfter.Add(npc);
                }
                else if (!delay.UntilEgoMove && Time.fixedTime >= delay.DelayAmount)
                {
                    List<string> route = entry.Value.Item2;
                    var routeLanes = parseLanes(route);
                    npcVehicleSimulator.Register(npc, routeLanes, entry.Value.Item3, entry.Value.Item4, entry.Value.Item5);
                    removeAfter.Add(npc);
                }
            }
            foreach(var npc in removeAfter)
                delayingMoveNPCs.Remove(npc);

            List<DelayingNPCVehicle> removeAfter2 = new List<DelayingNPCVehicle>();
            foreach(var entry in delayingSpawnNPCs)
            {
                NPCSpawnDelay delay = entry.Delay;
                if (delay.UntilEgoMove && autowareEgoTimer >= delay.DelayAmount)
                {
                    SpawnNPC(entry.VehiclePrefab, entry.SpawnPosition, entry.Route, entry.DesiredSpeeds, entry.Goal);
                    removeAfter2.Add(entry);
                }
                else if (!delay.UntilEgoMove && Time.fixedTime >= delay.DelayAmount)
                {
                    SpawnNPC(entry.VehiclePrefab, entry.SpawnPosition, entry.Route, entry.DesiredSpeeds, entry.Goal);
                    removeAfter2.Add(entry);
                }
            }
            foreach (var entry in removeAfter2)
                delayingSpawnNPCs.Remove(entry);
        }

        // spawn an obstacle
        private NPCVehicle PoseObstacle(string trafficLaneName, float distance)
        {
            GameObject obj = GameObject.Find(trafficLaneName);
            if (obj == null)
                throw new UnityException("[NPCSim] Cannot find traffic line with name: " + trafficLaneName);
            TrafficLane lane = obj.GetComponent<TrafficLane>();
            Vector3 position = NPCSimUtils.CalculatePosition(lane, distance, out int waypointIndex);
            Vector3 fwd = waypointIndex == 0 ?
                lane.Waypoints[1] - lane.Waypoints[0] :
                lane.Waypoints[waypointIndex] - lane.Waypoints[waypointIndex - 1];
            GameObject npc = Instantiate(npcTaxi,
                position,
                Quaternion.LookRotation(fwd));
            npc.name = "NPC-Taxi";
            var vehicle = npc.GetComponent<NPCVehicle>();
            vehicle.VehicleID = SpawnIdGenerator.Generate();
            return vehicle;
        }

        private TrafficLane parseLanes(string laneName)
        {
            GameObject obj = GameObject.Find(laneName);
            if (obj == null)
                throw new UnityException("[NPCSim] Cannot find traffic line with name: " + laneName);
            return obj.GetComponent<TrafficLane>();
        }
        private List<TrafficLane> parseLanes(List<string> laneNames)
        {
            var lanes = new List<TrafficLane>();
            foreach (string laneName in laneNames)
                lanes.Add(parseLanes(laneName));
            return lanes;
        }

        // spawn an NPC
        private NPCVehicle SpawnNPC(GameObject vehiclePrefab, LanePosition spawnPosition, out int waypointIndex)
        {
            // calculate position
            TrafficLane spawnLane = parseLanes(spawnPosition.LaneName);
            Vector3 position = NPCSimUtils.CalculatePosition(spawnLane, spawnPosition.Position, out waypointIndex);
            NPCVehicleSpawnPoint spawnPoint = new NPCVehicleSpawnPoint(spawnLane, position, waypointIndex);

            // spawn NPC
            var npc = npcVehicleSpawner.Spawn(vehiclePrefab, SpawnIdGenerator.Generate(), spawnPoint,
                Quaternion.LookRotation(spawnPoint.Forward));
            return npc;
        }

        // spawn an NPC and make it move
        private NPCVehicle SpawnNPC(GameObject vehiclePrefab, LanePosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LanePosition goal)
        {
            // spawn NPC
            NPCVehicle npc = SpawnNPC(vehiclePrefab, spawnPosition, out int waypointIndex);

            // set route and goal
            var routeLanes = parseLanes(route);
            npcVehicleSimulator.Register(npc, routeLanes, waypointIndex, desiredSpeeds, goal);
            return npc;
        }

        // spawn an NPC and delay `delay.ActivateDelay` seconds to make it move
        private NPCVehicle SpawnNPCAndDelayMovement(GameObject vehiclePrefab, LanePosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LanePosition goal, NPCSpawnDelay delay)
        {
            // spawn NPC
            NPCVehicle npc = SpawnNPC(vehiclePrefab, spawnPosition, out int waypointIndex);

            if(delay != null)
                delayingMoveNPCs.Add(npc, Tuple.Create<NPCSpawnDelay, List<string>, int, Dictionary<string, float>, LanePosition>
                (
                    delay,
                    route,
                    waypointIndex,
                    desiredSpeeds,
                    goal
                ));
            return npc;
        }

        // delay `delay.SpawnDelay` seconds to spawn the NPC
        private DelayingNPCVehicle SpawnNPCWithDelay(GameObject vehiclePrefab, LanePosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LanePosition goal, NPCSpawnDelay delay)
        {
            if (delay == null)
                throw new UnityException("[NPCSim]: Invalid NPCSpawnDelay paramater.");
            var delaynpc = new DelayingNPCVehicle(
                    SpawnIdGenerator.Generate(),
                    vehiclePrefab,
                    spawnPosition,
                    route,
                    desiredSpeeds,
                    goal,
                    delay);
            delayingSpawnNPCs.Add(delaynpc);
            return delaynpc;
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
            // TODO: make it possible to stop at the end of a lane
            var goal = new LanePosition("TrafficLane.265", 60f);

            SpawnNPC(npcTaxi, spawnPosition, route, desiredSpeeds, goal);
        }

        // another scenario
        private void Scenario2()
        {
            NPCVehicle npc = PoseObstacle("TrafficLane.263", 20f);
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

            SpawnNPC(npcTaxi, spawnPosition, route, desiredSpeeds, goal);

            // an obstacle
            PoseObstacle("TrafficLane.263", 15f);
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
            // desired speeds, defined for each lane
            var desiredSpeeds = new Dictionary<string, float>(){};
            // set goal
            // stop on lane 265, 40m far from the starting point of the lane
            var goal = new LanePosition("TrafficLane.240", 30f);

            SpawnNPCAndDelayMovement(npcTaxi, spawnPosition, route, desiredSpeeds, goal, NPCSpawnDelay.Delay(5f));
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

            SpawnNPCWithDelay(npcTaxi, spawnPosition, route, desiredSpeeds, goal, NPCSpawnDelay.Delay(5f));
        }
    }

    public class LanePosition
    {
        public string LaneName { get; set; }
        // distance from the starting point of the lane
        public float Position { get; set; }
        public LanePosition(string laneName, float position)
        {
            LaneName = laneName;
            Position = position;
        }
    }
}
