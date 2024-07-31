using System;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.TrafficSimulation;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class CustomNPCSpawningManager
    {
        // singleton instance
        private static CustomNPCSpawningManager manager;

        private GameObject autowareEgoCar;
        private GameObject npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan;
        private LayerMask vehicleLayerMask;
        private LayerMask groundLayerMask;
        private GameObject parentGameObject;
        private TrafficLane[] allTrafficLanes;

        private Rigidbody egoRigidbody;

        private NPCVehicleSimulator npcVehicleSimulator;
        private NPCVehicleSpawner npcVehicleSpawner;

        // the moment when Ego vehicle starts moving
        private float egoStartMovingTime = -1;
        // the moment when Ego vehicle gets plan trajectory
        private float egoEngagedTime = -1;

        // this flag becomes true when the ego vehicle got a plan trajectory
        private bool egoEngaged = false;

        private Dictionary<NPCVehicle, Tuple<NPCSpawnDelay, List<string>, int, Dictionary<string, float>, LaneOffsetPosition>> delayingMoveNPCs;
        private List<DelayingNPCVehicle> delayingSpawnNPCs;

        // all NPC vehicles spawned
        private List<NPCVehicle> npcs;

        private CustomNPCSpawningManager(GameObject parent, TrafficLane[] trafficLanes,
            GameObject ego, GameObject taxi, GameObject hatchback, GameObject smallCar,
            GameObject truck, GameObject van,
            LayerMask vehicleLM, LayerMask groundLM)
        {
            autowareEgoCar = ego;
            allTrafficLanes = trafficLanes;
            npcTaxi = taxi;
            npcHatchback = hatchback;
            npcSmallCar = smallCar;
            npcTruck = truck;
            npcVan = van;
            vehicleLayerMask = vehicleLM;
            groundLayerMask = groundLM;
            parentGameObject = parent;
            egoStartMovingTime = -1;
            egoEngagedTime = -1;
            egoEngaged = false;
            delayingMoveNPCs = new Dictionary<NPCVehicle, Tuple<NPCSpawnDelay, List<string>, int, Dictionary<string, float>, LaneOffsetPosition>>();
            delayingSpawnNPCs = new List<DelayingNPCVehicle>();
            npcs = new List<NPCVehicle>();

            egoRigidbody = autowareEgoCar.GetComponent<Rigidbody>();
            if (egoRigidbody == null)
                throw new UnityException("[NPCSim] Cannot find rigidbody of the ego vehicle");

            // only single NPCVehicleConfig is supported for all NPCs
            // TODO: support different NPCVehicleConfig for different NPCs
            NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();
            vehicleConfig.Acceleration = 7f;
            vehicleConfig.Deceleration = 8f;
            npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, 10, autowareEgoCar);
            npcVehicleSpawner = new NPCVehicleSpawner(parentGameObject, new GameObject[] { }, new TrafficLane[] { });

            try
            {
                SimulatorROS2Node.CreateSubscription<autoware_vehicle_msgs.msg.Engage>(
                    "/autoware/engage", msg =>
                    {
                        if (msg.Engage_)
                        {
                            Debug.Log("[NPCSim] Got /autoware/engage message: " + msg);
                            egoEngaged = true;
                        }
                    });
            }
            catch (NullReferenceException e)
            {
                Debug.LogError("[NPCSim] Cannot create ROS subscriber /autoware/engage. " +
                    "Make sure Autoware has been started. Exception detail: " + e);
            }
        }

        public static CustomNPCSpawningManager Initialize(GameObject parent, TrafficLane[] trafficLanes,
            GameObject ego, GameObject taxi, GameObject hatchback,
            GameObject smallCar, GameObject truck, GameObject van,
            LayerMask vehicleLM, LayerMask groundLM)
        {
            manager = new CustomNPCSpawningManager(parent, trafficLanes,
                ego, taxi, hatchback, smallCar, truck, van,
                vehicleLM, groundLM);
            return manager;
        }
        public static CustomNPCSpawningManager Manager() => manager;
        public static List<NPCVehicle> GetNPCs() => Manager().npcs;
        public static TrafficLane[] GetAllTrafficLanes() => Manager().allTrafficLanes;

        // this should be called every frame
        public void UpdateNPCs()
        {
            if (Manager() != null)
            {
                // check if Ego moved
                // TODO: use proper implementation instead of egoRigidbody.velocity
                if (egoStartMovingTime <= 0 &&
                    CustomSimUtils.DistanceIgnoreYAxis(egoRigidbody.velocity, Vector3.zero) > 0.2f)
                    egoStartMovingTime = Time.fixedTime;
                // check if Ego got trajectory
                if (egoEngagedTime <= 0 && egoEngaged)
                    egoEngagedTime = Time.fixedTime;

                npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);
                UpdateDelayingNPCs();
            }
        }

        /// <summary>
        /// Mainly perform 2 tasks:
        /// 1. Scan `delayingMoveNPCs`, i.e., the list of NPCs were spawned but not yet moved,
        ///    to make them move if they are ready to move.
        /// 2. Scan `delayingSpawnNPCs`, i.e., the list of NPCs waiting to be spawned,
        ///    to spawn them if they are ready to be spawned.
        /// </summary>
        private void UpdateDelayingNPCs()
        {
            List<NPCVehicle> removeAfter = new List<NPCVehicle>();
            foreach (var entry in delayingMoveNPCs)
            {
                NPCVehicle npc = entry.Key;
                NPCSpawnDelay delay = entry.Value.Item1;

                if ((delay.DelayType == NPCDelayType.UNTIL_EGO_MOVE && Time.fixedTime - egoStartMovingTime >= delay.DelayAmount) ||
                    (delay.DelayType == NPCDelayType.FROM_BEGINNING && Time.fixedTime >= delay.DelayAmount) ||
                    (delay.DelayType == NPCDelayType.UNTIL_EGO_ENGAGE && egoEngaged && Time.fixedTime - egoEngagedTime >= delay.DelayAmount))
                {
                    List<string> route = entry.Value.Item2;
                    var routeLanes = CustomSimUtils.ParseLanes(route);
                    npcVehicleSimulator.Register(npc, routeLanes, entry.Value.Item3, entry.Value.Item4, entry.Value.Item5);
                    removeAfter.Add(npc);
                }
            }
            foreach (var npc in removeAfter)
                delayingMoveNPCs.Remove(npc);

            List<DelayingNPCVehicle> removeAfter2 = new List<DelayingNPCVehicle>();
            foreach (var entry in delayingSpawnNPCs)
            {
                NPCSpawnDelay delay = entry.Delay;
                if ((delay.DelayType == NPCDelayType.UNTIL_EGO_MOVE && Time.fixedTime - egoStartMovingTime >= delay.DelayAmount) ||
                    (delay.DelayType == NPCDelayType.FROM_BEGINNING && Time.fixedTime >= delay.DelayAmount) ||
                    (delay.DelayType == NPCDelayType.UNTIL_EGO_ENGAGE && egoEngaged && Time.fixedTime - egoEngagedTime >= delay.DelayAmount))
                {
                    SpawnNPC(entry.VehicleType, entry.SpawnPosition, entry.Route, entry.DesiredSpeeds, entry.Goal);
                    removeAfter2.Add(entry);
                }
            }
            foreach (var entry in removeAfter2)
                delayingSpawnNPCs.Remove(entry);
        }

        // spawn a stand still vehicle 
        public static NPCVehicle PoseObstacle(string vehicleType, string trafficLaneName, float distance)
        {
            EnsureNonNullInstance(Manager());
            GameObject obj = GameObject.Find(trafficLaneName);
            if (obj == null)
                throw new UnityException("[NPCSim] Cannot find traffic line with name: " + trafficLaneName);
            TrafficLane lane = obj.GetComponent<TrafficLane>();
            Vector3 position = CustomSimUtils.CalculatePosition(lane, distance, out int waypointIndex);
            Vector3 fwd = waypointIndex == 0 ?
                lane.Waypoints[1] - lane.Waypoints[0] :
                lane.Waypoints[waypointIndex] - lane.Waypoints[waypointIndex - 1];
            GameObject npc = UnityEngine.Object.Instantiate(Manager().GetNPCPrefab(vehicleType),
                position,
                Quaternion.LookRotation(fwd));
            npc.name = "NPC-Taxi";
            var vehicle = npc.GetComponent<NPCVehicle>();
            vehicle.VehicleID = SpawnIdGenerator.Generate();
            GetNPCs().Add(vehicle);
            return vehicle;
        }

        // spawn an NPC
        public static NPCVehicle SpawnNPC(string vehicleType, LaneOffsetPosition spawnPosition, out int waypointIndex)
        {
            EnsureNonNullInstance(Manager());
            // calculate position
            TrafficLane spawnLane = CustomSimUtils.ParseLane(spawnPosition.GetLane());
            Vector3 position = CustomSimUtils.CalculatePosition(spawnLane, spawnPosition.GetOffset(), out waypointIndex);
            NPCVehicleSpawnPoint spawnPoint = new NPCVehicleSpawnPoint(spawnLane, position, waypointIndex);

            // spawn NPC
            var npc = Manager().npcVehicleSpawner.Spawn(Manager().GetNPCPrefab(vehicleType), SpawnIdGenerator.Generate(), spawnPoint,
                Quaternion.LookRotation(spawnPoint.Forward));
            GetNPCs().Add(npc);
            return npc;
        }

        // spawn an NPC and make it move
        public static NPCVehicle SpawnNPC(string vehicleType, LaneOffsetPosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LaneOffsetPosition goal)
        {
            // spawn NPC
            NPCVehicle npc = SpawnNPC(vehicleType, spawnPosition, out int waypointIndex);

            // set route and goal
            var routeLanes = CustomSimUtils.ParseLanes(route);
            Manager().npcVehicleSimulator.Register(npc, routeLanes, waypointIndex, desiredSpeeds, goal);
            return npc;
        }

        // spawn an NPC and delay `delay.ActivateDelay` seconds to make it move
        public static NPCVehicle SpawnNPCAndDelayMovement(string vehicleType, LaneOffsetPosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LaneOffsetPosition goal, NPCSpawnDelay delay)
        {
            // spawn NPC
            NPCVehicle npc = SpawnNPC(vehicleType, spawnPosition, out int waypointIndex);

            if (delay != null)
                Manager().delayingMoveNPCs.Add(npc, Tuple.Create<NPCSpawnDelay, List<string>, int, Dictionary<string, float>, LaneOffsetPosition>
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
        public static DelayingNPCVehicle SpawnNPCWithDelay(string vehicleType, LaneOffsetPosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LaneOffsetPosition goal, NPCSpawnDelay delay)
        {
            if (delay == null)
                throw new UnityException("[NPCSim]: Invalid NPCSpawnDelay paramater.");
            var delaynpc = new DelayingNPCVehicle(
                    SpawnIdGenerator.Generate(),
                    vehicleType,
                    spawnPosition,
                    route,
                    desiredSpeeds,
                    goal,
                    delay);
            Manager().delayingSpawnNPCs.Add(delaynpc);
            return delaynpc;
        }

        // parse vehicle type
        private GameObject GetNPCPrefab(string vehicleType)
        {
            switch (vehicleType.ToLower())
            {
                case "taxi":
                    return npcTaxi;
                case "hatchback":
                    return npcHatchback;
                case "small-car": case "smallcar":
                    return npcSmallCar;
                case "truck":
                    return npcTruck;
                case "van":
                    return npcVan;
                default:
                    Debug.LogWarning("[NPCSim] Cannot detect the vehicle type `" + vehicleType + "`. " +
                        "Use `taxi` as the default.");
                    return npcTaxi;
            }
        }
        private static void EnsureNonNullInstance(CustomNPCSpawningManager instance)
        {
            if (instance == null)
            {
                Debug.Log("[NPCSim] Could not find an instance of `CustomNPCSpawningManager`.");
                throw new NullReferenceException("[NPCSim] Could not find an instance of `CustomNPCSpawningManager`. " +
                    "Initialize it with `CustomNPCSpawningManager.Initialize()`");
            }
        }
    }
}