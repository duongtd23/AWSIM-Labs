using System;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.TrafficSimulation;
using AWSIM_Script.Object;
using AWSIM_Script.Error;
using AWSIM.AWAnalysis.Error;
using ROS2;

namespace AWSIM.AWAnalysis.CustomSim
{
    // TODO: check the route given is valid, i.e.,
    // two consecutive lanes are always valid
    public class CustomNPCSpawningManager
    {
        // singleton instance
        private static CustomNPCSpawningManager manager;

        private GameObject autowareEgoCar;
        private Vehicle _egoVehicle;
        private GameObject npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan;
        private LayerMask vehicleLayerMask;
        private LayerMask groundLayerMask;
        private GameObject parentGameObject;
        private TrafficLane[] allTrafficLanes;

        private NPCVehicleSimulator npcVehicleSimulator;
        private NPCVehicleSpawner npcVehicleSpawner;

        // the moment when Ego vehicle starts moving
        private float egoStartMovingTime = -1;
        // the moment when Ego vehicle gets plan trajectory
        private float egoEngagedTime = -1;

        // this flag becomes true when the ego vehicle got a plan trajectory
        private bool egoEngaged = false;

        // 2nd item: waypoint index
        private Dictionary<NPCVehicle, Tuple<int, NPCCar>> delayingMoveNPCs;
        private List<NPCCar> delayingSpawnNPCs;

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
            delayingMoveNPCs = new Dictionary<NPCVehicle, Tuple<int, NPCCar>>();
            delayingSpawnNPCs = new List<NPCCar>();
            npcs = new List<NPCVehicle>();

            _egoVehicle = autowareEgoCar.GetComponent<Vehicle>();

            NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();
            vehicleConfig.Acceleration = ConfigLoader.Config().NpcAcceleration;
            vehicleConfig.Deceleration = ConfigLoader.Config().NpcDeceleration;
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

        public static CustomNPCSpawningManager Manager()
        {
            while (manager == null) ;
            return manager;
        }

        public static List<NPCVehicle> GetNPCs() => Manager().npcs;
        public static TrafficLane[] GetAllTrafficLanes() => Manager().allTrafficLanes;

        // this should be called every frame
        public void UpdateNPCs()
        {
            if (Manager() != null)
            {
                // check if Ego moved
                if (egoStartMovingTime <= 0 &&
                    CustomSimUtils.MagnitudeIgnoreYAxis(_egoVehicle.Velocity) > 0.15f)
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
                NPCVehicle npcVehicle = entry.Key;
                NPCCar npcCar = entry.Value.Item2;
                int waypointIndex = entry.Value.Item1;
                NPCSpawnDelay delay = npcCar.SpawnDelayOption;

                if ((delay.DelayType == DelayKind.UNTIL_EGO_MOVE && Time.fixedTime - egoStartMovingTime >= delay.DelayAmount) ||
                    (delay.DelayType == DelayKind.FROM_BEGINNING && Time.fixedTime >= delay.DelayAmount) ||
                    (delay.DelayType == DelayKind.UNTIL_EGO_ENGAGE && egoEngaged && Time.fixedTime - egoEngagedTime >= delay.DelayAmount))
                {
                    npcVehicleSimulator.Register(npcVehicle, waypointIndex, 
                        CustomSimUtils.ValidateGoal(npcCar.Goal),
                        npcCar.Config);
                    removeAfter.Add(npcVehicle);
                }
            }
            foreach (var npc in removeAfter)
                delayingMoveNPCs.Remove(npc);

            List<NPCCar> removeAfter2 = new List<NPCCar>();
            foreach (var npcCar in delayingSpawnNPCs)
            {
                NPCSpawnDelay delay = npcCar.SpawnDelayOption;
                if ((delay.DelayType == DelayKind.UNTIL_EGO_MOVE && Time.fixedTime - egoStartMovingTime >= delay.DelayAmount) ||
                    (delay.DelayType == DelayKind.FROM_BEGINNING && Time.fixedTime >= delay.DelayAmount) ||
                    (delay.DelayType == DelayKind.UNTIL_EGO_ENGAGE && egoEngaged && Time.fixedTime - egoEngagedTime >= delay.DelayAmount))
                {
                    SpawnNPC(npcCar.VehicleType, npcCar.InitialPosition, npcCar.Config, npcCar.Goal, npcCar.Name);
                    removeAfter2.Add(npcCar);
                }
            }
            foreach (var entry in removeAfter2)
                delayingSpawnNPCs.Remove(entry);
        }

        // spawn a stand still vehicle 
        public static NPCVehicle PoseObstacle(VehicleType vehicleType, IPosition spawnPosition, string name = "")
        {
            EnsureNonNullInstance(Manager());
            TrafficLane lane = CustomSimUtils.ParseLane(spawnPosition.GetLane());
            Vector3 position = CustomSimUtils.CalculatePosition(
                lane, spawnPosition.GetOffset(), out int waypointIndex);
            Vector3 fwd = waypointIndex == 0 ?
                lane.Waypoints[1] - lane.Waypoints[0] :
                lane.Waypoints[waypointIndex] - lane.Waypoints[waypointIndex - 1];
            GameObject npcGameObj = UnityEngine.Object.Instantiate(Manager().GetNPCPrefab(vehicleType),
                position,
                Quaternion.LookRotation(fwd));
            NPCVehicle npc = npcGameObj.GetComponent<NPCVehicle>();
            npc.VehicleID = SpawnIdGenerator.Generate();
            if (name != "")
                npc.ScriptName = name;
            GetNPCs().Add(npc);
            return npc;
        }

        public static NPCVehicle SpawnNPC(VehicleType vehicleType, IPosition spawnPosition, string name = "")
        {
            return PoseObstacle(vehicleType, spawnPosition, name);
        }

        // spawn an NPC (static, no movement)
        private static NPCVehicle SpawnNPC(VehicleType vehicleType, IPosition spawnPosition, out int waypointIndex,
            string name = "")
        {
            EnsureNonNullInstance(Manager());
            // calculate position
            TrafficLane spawnLane = CustomSimUtils.ParseLane(spawnPosition.GetLane());
            Vector3 position = CustomSimUtils.CalculatePosition(
                spawnLane, spawnPosition.GetOffset(), out waypointIndex);
            NPCVehicleSpawnPoint spawnPoint = new NPCVehicleSpawnPoint(spawnLane, position, waypointIndex);

            // spawn NPC
            NPCVehicle npc = Manager().npcVehicleSpawner.Spawn(Manager().GetNPCPrefab(vehicleType),
                SpawnIdGenerator.Generate(), spawnPoint,
                Quaternion.LookRotation(spawnPoint.Forward));
            if (name != "")
                npc.ScriptName = name;
            GetNPCs().Add(npc);
            return npc;
        }

        // spawn an NPC and make it move
        // npcConfig.Route $ RouteAndSpeeds must be non-null
        public static NPCVehicle SpawnNPC(VehicleType vehicleType, IPosition spawnPosition,
            NPCConfig npcConfig, IPosition goal, string name = "")
        {
            // spawn NPC
            NPCVehicle npc = SpawnNPC(vehicleType, spawnPosition, out int waypointIndex, name);
            Manager().npcVehicleSimulator.Register(npc, waypointIndex,
                CustomSimUtils.ValidateGoal(goal),
                npcConfig);
            return npc;
        }

        // spawn an NPC and delay `delay.ActivateDelay` seconds to make it move
        public static NPCVehicle SpawnNPCAndDelayMovement(NPCCar npcCar)
        {
            if (npcCar.SpawnDelayOption == null || npcCar.SpawnDelayOption.ActionDelayed != DelayedAction.MOVING)
                throw new CustomSimException("[AWAnalysis]: Invalid NPCSpawnDelay paramater.");

            // spawn NPC
            NPCVehicle npc = SpawnNPC(npcCar.VehicleType, npcCar.InitialPosition, out int waypointIndex, npcCar.Name);

            Manager().delayingMoveNPCs.Add(npc,
                Tuple.Create(waypointIndex, npcCar));
            return npc;
        }

        public static void SpawnNPC(NPCCar npcCar)
        {
            if (npcCar.InitialPosition == null)
                throw new InvalidScriptException("Undefined initial position" + 
                                                 (npcCar.Name == null ? "." : " " + npcCar.Name));
            ValidateNPC(ref npcCar);
            if (!npcCar.HasGoal())
            {
                PoseObstacle(npcCar.VehicleType, npcCar.InitialPosition, npcCar.Name);
            }
            else
            {
                if (npcCar.HasDelayOption())
                {
                    switch (npcCar.SpawnDelayOption.ActionDelayed)
                    {
                        case DelayedAction.SPAWNING:
                            Manager().delayingSpawnNPCs.Add(npcCar);
                            break;
                        case DelayedAction.MOVING:
                            SpawnNPCAndDelayMovement(npcCar);
                            break;
                    }
                }
                else
                    SpawnNPC(npcCar.VehicleType, npcCar.InitialPosition,
                        npcCar.Config, npcCar.Goal, npcCar.Name);
            }
        }

        // validate (and update if neccessary) a given NPC
        // the given NPC might lack of route, etc.
        private static void ValidateNPC(ref NPCCar npcCar)
        {
            // validate spawn lane
            TrafficLane spawnLane = CustomSimUtils.ParseLane(npcCar.InitialPosition.GetLane());
            if (!npcCar.HasGoal())
                return;
            // validate goal lane if exists
            TrafficLane goalLane = CustomSimUtils.ParseLane(npcCar.Goal.GetLane());
            // validate route
            var npcConfig = npcCar.Config;
            // if there is no route config, validate if goal can be reached directly from spawn lane
            if (npcConfig?.RouteAndSpeeds == null || npcConfig.RouteAndSpeeds.Count == 0)
            {
                if (spawnLane == goalLane)
                    npcCar.Config = new NPCConfig(new List<string> {
                        npcCar.InitialPosition.GetLane()
                    });
                else if (spawnLane.NextLanes.Contains(goalLane))
                {
                    npcCar.Config = new NPCConfig(new List<string> {
                        npcCar.InitialPosition.GetLane(),
                        npcCar.Goal.GetLane()
                    });
                }
                else
                    throw new InvalidScriptException($"Undefined route from {npcCar.InitialPosition} to {npcCar.Goal}.");
            }
        }

        private GameObject GetNPCPrefab(VehicleType vehicleType)
        {
            switch (vehicleType)
            {
                case VehicleType.TAXI:
                    return npcTaxi;
                case VehicleType.HATCHBACK:
                    return npcHatchback;
                case VehicleType.SMALL_CAR:
                    return npcSmallCar;
                case VehicleType.TRUCK:
                    return npcTruck;
                case VehicleType.VAN:
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