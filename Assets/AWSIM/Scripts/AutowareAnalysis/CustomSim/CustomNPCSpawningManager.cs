using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using AWSIM.TrafficSimulation;
using AWSIM_Script.Object;
using AWSIM_Script.Error;
using AWSIM.AWAnalysis.Error;
using AWSIM.AWAnalysis.TraceExporter.Objects;

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
        private GameObject _casualPedestrian, _elegantPedestrian;
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
        private List<NPCPedes> _delayingSpawnPedestrians;

        // all NPCs spawned
        private List<NPCVehicle> npcs;
        private List<Tuple<NPCPedes, NPCPedestrian>> _pedestrians;

        #region constructor and public update
        
        private CustomNPCSpawningManager(GameObject parent, TrafficLane[] trafficLanes,
            GameObject ego, GameObject taxi, GameObject hatchback, GameObject smallCar,
            GameObject truck, GameObject van,
            GameObject casualPedestrian, GameObject elegantPedestrian,
            LayerMask vehicleLM, LayerMask groundLM)
        {
            autowareEgoCar = ego;
            allTrafficLanes = trafficLanes;
            npcTaxi = taxi;
            npcHatchback = hatchback;
            npcSmallCar = smallCar;
            npcTruck = truck;
            npcVan = van;
            _casualPedestrian = casualPedestrian;
            _elegantPedestrian = elegantPedestrian;
            vehicleLayerMask = vehicleLM;
            groundLayerMask = groundLM;
            parentGameObject = parent;
            egoStartMovingTime = -1;
            egoEngagedTime = -1;
            egoEngaged = false;
            delayingMoveNPCs = new Dictionary<NPCVehicle, Tuple<int, NPCCar>>();
            delayingSpawnNPCs = new List<NPCCar>();
            _delayingSpawnPedestrians = new List<NPCPedes>();
            npcs = new List<NPCVehicle>();
            _pedestrians = new List<Tuple<NPCPedes, NPCPedestrian>>();

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
                            Debug.Log("[AWAnalysis] Got /autoware/engage message: " + msg);
                            egoEngaged = true;
                        }
                    });
            }
            catch (NullReferenceException e)
            {
                Debug.LogError("[AWAnalysis] Cannot create ROS subscriber /autoware/engage. " +
                    "Make sure Autoware has been started. Exception detail: " + e);
            }
        }

        public static CustomNPCSpawningManager Initialize(GameObject parent, TrafficLane[] trafficLanes,
            GameObject ego, GameObject taxi, GameObject hatchback,
            GameObject smallCar, GameObject truck, GameObject van,
            GameObject casualPedestrian, GameObject elegantPedestrian,
            LayerMask vehicleLM, LayerMask groundLM)
        {
            manager = new CustomNPCSpawningManager(parent, trafficLanes,
                ego, taxi, hatchback, smallCar, truck, van,
                casualPedestrian, elegantPedestrian,
                vehicleLM, groundLM);
            return manager;
        }

        public static CustomNPCSpawningManager Manager()
        {
            while (manager == null) ;
            return manager;
        }

        public static List<NPCVehicle> GetNPCs() => Manager().npcs;
        public static List<Tuple<NPCPedes, NPCPedestrian>> GetPedestrians() => Manager()._pedestrians;
        
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
                
                // update positions of pedestrians
                UpdatePedestrians();
                
                UpdateDelayingNPCs();
                
                // update delaying pedestrians
                UpdateDelayingPedestrians();

                UpdateCutoutLeadingNPC();

                UpdateDecelerationNPC();
            }
        }
        
        #endregion

        #region Inner computation
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
                INPCSpawnDelay idelay = npcCar.SpawnDelayOption;

                if (idelay is NPCDelayTime delayTime)
                {
                    if ((delayTime.DelayType == DelayKind.UNTIL_EGO_MOVE &&
                         Time.fixedTime - egoStartMovingTime >= delayTime.DelayAmount) ||
                        (delayTime.DelayType == DelayKind.FROM_BEGINNING && Time.fixedTime >= delayTime.DelayAmount) ||
                        (delayTime.DelayType == DelayKind.UNTIL_EGO_ENGAGE && egoEngaged &&
                         Time.fixedTime - egoEngagedTime >= delayTime.DelayAmount))
                    {
                        npcVehicleSimulator.Register(npcVehicle, waypointIndex,
                            npcCar.Goal,
                            npcCar.Config);
                        removeAfter.Add(npcVehicle);
                    }
                }
                else if (idelay is NPCDelayDistance delayDistance)
                {
                    if (_egoVehicle.Velocity.magnitude > 0.1f &&
                        CustomSimUtils.LongitudeDistance(
                            _egoVehicle.Position,
                            _egoVehicle.Rotation * Vector3.forward,
                            npcVehicle.Position) <= delayDistance.Distance)
                    {
                        npcVehicleSimulator.Register(npcVehicle, waypointIndex, 
                            npcCar.Goal,
                            npcCar.Config);
                        removeAfter.Add(npcVehicle);
                    }
                }
            }
            foreach (var npc in removeAfter)
                delayingMoveNPCs.Remove(npc);

            List<NPCCar> removeAfter2 = new List<NPCCar>();
            foreach (var npcCar in delayingSpawnNPCs)
            {
                INPCSpawnDelay idelay = npcCar.SpawnDelayOption;
                if (idelay is NPCDelayTime delayTime)
                {
                    if ((delayTime.DelayType == DelayKind.UNTIL_EGO_MOVE &&
                         Time.fixedTime - egoStartMovingTime >= delayTime.DelayAmount) ||
                        (delayTime.DelayType == DelayKind.FROM_BEGINNING && Time.fixedTime >= delayTime.DelayAmount) ||
                        (delayTime.DelayType == DelayKind.UNTIL_EGO_ENGAGE && egoEngaged &&
                         Time.fixedTime - egoEngagedTime >= delayTime.DelayAmount))
                    {
                        if (npcCar.HasGoal())
                            SpawnNPC(npcCar.VehicleType, npcCar.InitialPosition, npcCar.Config, npcCar.Goal, npcCar.Name);
                        else
                            PoseObstacle(npcCar.VehicleType, npcCar.InitialPosition, npcCar.Name);
                        removeAfter2.Add(npcCar);
                    }
                }
            }
            foreach (var entry in removeAfter2)
                delayingSpawnNPCs.Remove(entry);
        }

        private void UpdateDelayingPedestrians()
        {
            List<NPCPedes> removeAfter = new List<NPCPedes>();
            for (int i = 0; i< _delayingSpawnPedestrians.Count; i++)
            {
                var delayPedestrian = _delayingSpawnPedestrians[i];
                var delayTime = delayPedestrian.Config.Delay;
                if ((delayTime.DelayType == DelayKind.UNTIL_EGO_MOVE &&
                     Time.fixedTime - egoStartMovingTime >= delayTime.DelayAmount) ||
                    (delayTime.DelayType == DelayKind.FROM_BEGINNING && 
                     Time.fixedTime >= delayTime.DelayAmount) ||
                    (delayTime.DelayType == DelayKind.UNTIL_EGO_ENGAGE && egoEngaged &&
                     Time.fixedTime - egoEngagedTime >= delayTime.DelayAmount))
                {
                    DoSpawnPedestrian(ref delayPedestrian);
                    removeAfter.Add(delayPedestrian);
                }
            }
            foreach (var entry in removeAfter)
                _delayingSpawnPedestrians.Remove(entry);
        }

        private void UpdatePedestrians()
        {
            foreach (var entry in _pedestrians)
            {
                NPCPedes npcPedes = entry.Item1;
                var delayTime = npcPedes.Config?.Delay;
                if (delayTime == null ||
                    (delayTime.DelayType == DelayKind.UNTIL_EGO_MOVE &&
                     Time.fixedTime - egoStartMovingTime >= delayTime.DelayAmount) ||
                    (delayTime.DelayType == DelayKind.FROM_BEGINNING &&
                     Time.fixedTime >= delayTime.DelayAmount) ||
                    (delayTime.DelayType == DelayKind.UNTIL_EGO_ENGAGE && egoEngaged &&
                     Time.fixedTime - egoEngagedTime >= delayTime.DelayAmount))
                {
                    var newPosition = npcPedes.LastPosition +
                                      npcPedes.LastRotation * Vector3.forward * (npcPedes.Config.Speed * Time.fixedDeltaTime);
                    if (CustomSimUtils.SignDistance(newPosition, npcPedes.CurrentWaypoint, npcPedes.LastRotation) > -0.02f)
                    {
                        npcPedes.LastPosition = newPosition;
                        entry.Item2.SetPosition(npcPedes.LastPosition);
                    }
                    // update the waypoint
                    else
                    {
                        // if this is the last waypoint
                        if (npcPedes.CurrentWaypointIndex == npcPedes.Waypoints.Count - 1)
                        {
                            // loop: turn backward
                            if (npcPedes.Config != null && npcPedes.Config.Loop)
                            {
                                
                            }
                            // reached the goal, do nothing
                            else
                            {
                                
                            }
                        }
                        // update rotation to match new waypoint
                        else
                        {
                            npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                            npcPedes.CurrentWaypointIndex++;
                            npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.CurrentWaypoint - npcPedes.LastPosition);
                            entry.Item2.SetRotation(npcPedes.LastRotation);
                        }
                    }
                }
            }
        }

        private void UpdateCutoutLeadingNPC()
        {
            var cutoutNPC = CutoutVehicle();
            if (cutoutNPC == null) return;
            
            var laneChangeConfig = cutoutNPC.CustomConfig.LaneChange as CutOutLaneChange;
            var leadingNPC =
                delayingSpawnNPCs.FirstOrDefault(npcCar => npcCar.Name == laneChangeConfig.LeadVehicle.Name);

            if (leadingNPC != null)
            {
                var internalState = npcVehicleSimulator.VehicleStates.FirstOrDefault(state =>
                    state.CustomConfig.HasALaneChange() &&
                    state.CustomConfig.LaneChange is CutOutLaneChange);
                if (internalState?.CurrentFollowingLane.name == laneChangeConfig.TargetLane &&
                    internalState?.WaypointIndex == laneChangeConfig.TargetLaneWaypointIndex &&
                    leadingNPC.SpawnDelayOption is NPCDelayTime
                    {
                        DelayAmount: NPCDelayTime.DUMMY_DELAY_AMOUNT
                    } delayTime)
                {
                    float desired_dx0 = Mathf.Min(_egoVehicle.Velocity.magnitude, cutoutNPC.Velocity.magnitude) *
                                        ConfigLoader.Config().TimeHeadWay;
                    float actual_dx0 = CustomSimUtils.DistanceIgnoreYAxis(_egoVehicle.Position, cutoutNPC.Position);
                    actual_dx0 -= (float)EgoSingletonInstance.GetEgoDetailInfo().extents.z +
                                  (float)EgoSingletonInstance.GetEgoDetailInfo().center.z;
                    actual_dx0 -= (float)cutoutNPC.GetCarInfo().extents.z -
                                  (float)cutoutNPC.GetCarInfo().center.z;
                    if (actual_dx0 > desired_dx0)
                    {
                        var leadingSpawnPos = new RelativePosition(leadingNPC.InitialPosition,
                            RelativePositionSide.FORWARD,
                            desired_dx0 - actual_dx0 +
                            (float)GetNPCCarInfo(leadingNPC.VehicleType).extents.z -
                            (float)GetNPCCarInfo(leadingNPC.VehicleType).center.z);
                        SpawnNPC(leadingNPC.VehicleType, leadingSpawnPos, out int wpId, leadingNPC.Name);
                    }
                    delayingSpawnNPCs.Remove(leadingNPC);
                }
            }
        }

        private void UpdateDecelerationNPC()
        {
            var decelNPC = DecelerationVehicle();
            if (decelNPC == null) return;
            
            var internalState = npcVehicleSimulator.VehicleStates.FirstOrDefault(state =>
                state.CustomConfig != null &&
                state.CustomConfig.AggresiveDrive &&
                state.CustomConfig.Deceleration >= 9.8f);
            if (internalState?.SpeedMode == NPCVehicleSpeedMode.STOP)
            {
                var unspawnNPC = delayingSpawnNPCs.FirstOrDefault(npcCar =>
                    npcCar.InitialPosition.Equals(LaneOffsetPosition.DummyPosition()) &&
                    npcCar.HasDelayOption() &&
                    npcCar.SpawnDelayOption.ActionDelayed == DelayedAction.SPAWNING &&
                    npcCar.SpawnDelayOption is NPCDelayTime delayTime &&
                    delayTime.DelayType == DelayKind.FROM_BEGINNING &&
                    delayTime.DelayAmount.Equals(NPCDelayTime.DUMMY_DELAY_AMOUNT));
                if (unspawnNPC != null)
                {
                    float desired_dx0 = Mathf.Min(_egoVehicle.Velocity.magnitude, decelNPC.Velocity.magnitude) *
                                        ConfigLoader.Config().TimeHeadWay;
                    float actual_dx0 = CustomSimUtils.DistanceIgnoreYAxis(_egoVehicle.Position, decelNPC.Position);
                    actual_dx0 -= (float)EgoSingletonInstance.GetEgoDetailInfo().extents.z +
                                  (float)EgoSingletonInstance.GetEgoDetailInfo().center.z;
                    actual_dx0 -= (float)decelNPC.GetCarInfo().extents.z -
                                  (float)decelNPC.GetCarInfo().center.z;

                    if (actual_dx0 > desired_dx0)
                    {
                        float stopDistance = 0.5f * decelNPC.Velocity.magnitude * decelNPC.Velocity.magnitude /
                                             decelNPC.CustomConfig.Deceleration;
                        float shiftBack = actual_dx0 - desired_dx0 - stopDistance;
                        Vector3 backDirection = decelNPC.Velocity * -1;
                        Vector3 spawnPosition = decelNPC.Position + backDirection.normalized * shiftBack;
                        PoseObstacle(unspawnNPC.VehicleType, spawnPosition, backDirection * -1, unspawnNPC.Name);
                    }
                    delayingSpawnNPCs.Remove(unspawnNPC);
                }
            }
        }

        private NPCVehicle CutoutVehicle()
        {
            var results = GetNPCs().FindAll(npc0 =>
                npc0.CustomConfig != null &&
                npc0.CustomConfig.HasALaneChange() &&
                npc0.CustomConfig.LaneChange is CutOutLaneChange);
            if (results.Count >= 2)
            {
                Debug.LogError("Found more than one possible cutout vehicle. Use the first one by default.");
            }
            return results.FirstOrDefault();
        }

        private NPCVehicle CutinVehicle()
        {
            var results = GetNPCs().FindAll(npc0 =>
                npc0.CustomConfig != null &&
                npc0.CustomConfig.HasALaneChange() &&
                npc0.CustomConfig.LaneChange is CutInLaneChange);
            if (results.Count >= 2)
            {
                Debug.LogError("Found more than one possible cutin vehicle. Use the first one by default.");
            }
            return results.FirstOrDefault();
        }
        
        private NPCVehicle DecelerationVehicle()
        {
            var results = GetNPCs().FindAll(npc0 =>
                npc0.CustomConfig != null &&
                npc0.CustomConfig.AggresiveDrive &&
                npc0.CustomConfig.Deceleration >= 9.8f);
            if (results.Count >= 2)
            {
                Debug.LogError("Found more than one possible deceleration vehicle. Use the first one by default.");
            }
            return results.FirstOrDefault();
        }
        
        #endregion
    
        public static NPCVehicle PoseObstacle(VehicleType vehicleType, Vector3 position, Vector3 forwardDirection, string name)
        {
            GameObject npcGameObj = UnityEngine.Object.Instantiate(Manager().GetNPCPrefab(vehicleType),
                position,
                Quaternion.LookRotation(forwardDirection));
            NPCVehicle npc = npcGameObj.GetComponent<NPCVehicle>();
            npc.VehicleID = SpawnIdGenerator.Generate();
            if (name != "")
                npc.ScriptName = name;
            GetNPCs().Add(npc);
            return npc;
        }
        
        // spawn an NPC (static, no movement)
        private static NPCVehicle SpawnNPC(VehicleType vehicleType, IPosition spawnPosition, out int waypointIndex,
            NPCConfig customConfig, string name = "")
        {
            var npc = SpawnNPC(vehicleType, spawnPosition, out waypointIndex, name);
            npc.CustomConfig = customConfig;
            return npc;
        }
        
        #region APIs for spawning NPCs     

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

        // spawn an NPC and let it move
        // npcConfig.Route $ RouteAndSpeeds must be non-null
        public static NPCVehicle SpawnNPC(VehicleType vehicleType, IPosition spawnPosition,
            NPCConfig npcConfig, IPosition goal, string name = "")
        {
            // spawn NPC
            NPCVehicle npc = SpawnNPC(vehicleType, spawnPosition, out int waypointIndex, npcConfig, name);
            Manager().npcVehicleSimulator.Register(npc, waypointIndex,
                ValidateGoal(goal),
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
            if (npcCar.Config != null)
                npc.CustomConfig = npcCar.Config;

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
                if (npcCar.HasDelayOption() && npcCar.SpawnDelayOption.ActionDelayed == DelayedAction.SPAWNING)
                    Manager().delayingSpawnNPCs.Add(npcCar);
                else
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
        
        #endregion
    
        #region APIs for creating pedestrians
        public static void SpawnPedestrian(NPCPedes pedestrian)
        {
            if (pedestrian.HasDelayOption())
            {
                switch (pedestrian.Config.Delay.ActionDelayed)
                {
                    case DelayedAction.SPAWNING:
                        Manager()._delayingSpawnPedestrians.Add(pedestrian);
                        break;
                    case DelayedAction.MOVING:
                        DoSpawnPedestrian(ref pedestrian);
                        break;
                }
            }
            else DoSpawnPedestrian(ref pedestrian);
        }

        private static NPCPedestrian DoSpawnPedestrian(ref NPCPedes npcPedes)
        {
            if (npcPedes.Waypoints?.Count < 2)
            {
                throw new InvalidScriptException("Please specify at least 2 waypoints for the pedestrian " + npcPedes.Name);
            }

            npcPedes.LastPosition = npcPedes.Waypoints[0];
            npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.Waypoints[1] - npcPedes.Waypoints[0]);
            GameObject pedesGameObj = UnityEngine.Object.Instantiate(Manager().GetNPCPrefab(npcPedes.PedType),
                npcPedes.LastPosition,
                npcPedes.LastRotation);
            NPCPedestrian pedestrian = pedesGameObj.GetComponent<NPCPedestrian>();
            GetPedestrians().Add(Tuple.Create(npcPedes,pedestrian));
            return pedestrian;
        }

        #endregion

        #region APIs for retrieve internal state

        public static NPCVehicle GetCutOutVehicle() => Manager().CutoutVehicle();
        public static NPCVehicle GetCutInVehicle() => Manager().CutinVehicle();
        public static NPCVehicle GetDecelerationVehicle() => Manager().DecelerationVehicle();
        public static NPCVehicleInternalState CutOutNPCInternalState() => 
            Manager().npcVehicleSimulator?.VehicleStates?.FirstOrDefault(state =>
                state.CustomConfig != null &&
                state.CustomConfig.HasALaneChange() &&
                state.CustomConfig.LaneChange is CutOutLaneChange);
        public static NPCVehicleInternalState CutInNPCInternalState() => 
            Manager().npcVehicleSimulator?.VehicleStates?.FirstOrDefault(state =>
                state.CustomConfig != null &&
                state.CustomConfig.HasALaneChange() &&
                state.CustomConfig.LaneChange is CutInLaneChange);
        public static NPCVehicleInternalState DecelerationNPCInternalState() => 
            Manager().npcVehicleSimulator?.VehicleStates?.FirstOrDefault(state =>
                state.CustomConfig != null &&
                state.CustomConfig.AggresiveDrive &&
                state.CustomConfig.Deceleration >= 9.8f);

        #endregion

        // validate (and update if neccessary) a given NPC
        // the given NPC might lack of route, etc.
        private static void ValidateNPC(ref NPCCar npcCar)
        {
            if (!npcCar.HasGoal()) return;

            // validate spawn lane
            TrafficLane spawnLane = CustomSimUtils.ParseLane(npcCar.InitialPosition.GetLane());
            
            // validate goal lane if exists
            TrafficLane goalLane = CustomSimUtils.ParseLane(npcCar.Goal.GetLane());
            npcCar.Goal = ValidateGoal(npcCar.Goal, goalLane);
            
            // validate route
            var npcConfig = npcCar.Config;
            if (npcConfig == null) return;
            // if there is no route config, validate if goal can be reached directly from spawn lane
            if (npcConfig.RouteAndSpeeds == null || npcConfig.RouteAndSpeeds.Count == 0)
            {
                if (spawnLane == goalLane)
                {
                    npcCar.Config.UpdateRouteAndSpeeds(new List<string>
                    {
                        npcCar.InitialPosition.GetLane()
                    });
                }
                else if (spawnLane.NextLanes.Contains(goalLane))
                {
                    npcCar.Config.UpdateRouteAndSpeeds(new List<string> {
                        npcCar.InitialPosition.GetLane(),
                        npcCar.Goal.GetLane()
                    });
                }
                else
                    throw new InvalidScriptException($"Undefined route from {npcCar.InitialPosition} to {npcCar.Goal}.");
            }

            if (npcCar.Config.HasALaneChange())
            {
                // update lane change config
                TrafficLane sourceLaneChange = CustomSimUtils.ParseLane(npcConfig.LaneChange.SourceLane);
                TrafficLane targetLaneChange = CustomSimUtils.ParseLane(npcConfig.LaneChange.TargetLane);

                if (npcConfig.LaneChange.LateralVelocity == 0)
                    npcConfig.LaneChange.LateralVelocity = ILaneChange.DEFAULT_LATERAL_VELOCITY;
                if (npcConfig.LaneChange.LongitudinalVelocity == 0)
                {
                    if (npcConfig.HasDesiredSpeed(npcConfig.LaneChange.SourceLane))
                        npcConfig.LaneChange.LongitudinalVelocity =
                            npcConfig.GetDesiredSpeed(npcConfig.LaneChange.SourceLane);
                    else
                        npcConfig.LaneChange.LongitudinalVelocity = sourceLaneChange.SpeedLimit;
                }

                bool leftLaneChange = CustomSimUtils.OnLeftSide(
                    targetLaneChange.Waypoints[0],
                    sourceLaneChange.Waypoints[0], sourceLaneChange.Waypoints[1]);
                npcConfig.LaneChange.ChangeDirection = leftLaneChange ? Side.LEFT : Side.RIGHT;
            }
        }
        
        // if the offset of goal exceeds the lane's total length,
        // set the offset to lane length
        private static IPosition ValidateGoal(IPosition goal)
        {
            TrafficLane lane = CustomSimUtils.ParseLane(goal.GetLane());
            if (goal.GetOffset() > lane.TotalLength())
            {
                return new LaneOffsetPosition(goal.GetLane(), lane.TotalLength());
            }
            return goal;
        }
        
        private static IPosition ValidateGoal(IPosition goal, TrafficLane goalLane)
        {
            if (goal.GetOffset() > goalLane.TotalLength())
            {
                Debug.Log("[AWAnalysis]: Goal offset exceeds lane's length. Use the end point of lane");
                return new LaneOffsetPosition(goal.GetLane(), goalLane.TotalLength());
            }
            return goal;
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
        
        private GameObject GetNPCPrefab(PedesType pedestrianType)
        {
            switch (pedestrianType)
            {
                case PedesType.CASUAL:
                    return _casualPedestrian;
                case PedesType.ELEGANT:
                    return _elegantPedestrian;
                default:
                    Debug.LogWarning("[NPCSim] Cannot parse the pedestrian type `" + pedestrianType + "`. " +
                                     "Use `casual` pedestrian as the default.");
                    return _casualPedestrian;
            }
        }

        public static NPCDetailObject GetNPCCarInfo(VehicleType vehicleType)
        {
            switch (vehicleType)
            {
                case VehicleType.TAXI:
                    return Manager().npcTaxi.GetComponent<NPCVehicle>().GetCarInfo();
                case VehicleType.HATCHBACK:
                    return Manager().npcHatchback.GetComponent<NPCVehicle>().GetCarInfo();
                case VehicleType.SMALL_CAR:
                    return Manager().npcSmallCar.GetComponent<NPCVehicle>().GetCarInfo();
                case VehicleType.TRUCK:
                    return Manager().npcTruck.GetComponent<NPCVehicle>().GetCarInfo();
                case VehicleType.VAN:
                    return Manager().npcVan.GetComponent<NPCVehicle>().GetCarInfo();
            }
            throw new InvalidScriptException("Cannot detect the vehicle type `" + vehicleType + "`.");
        }

        private static void EnsureNonNullInstance(CustomNPCSpawningManager instance)
        {
            if (instance == null)
            {
                Debug.Log("[AWAnalysis] Could not find an instance of `CustomNPCSpawningManager`.");
                throw new NullReferenceException("[NPCSim] Could not find an instance of `CustomNPCSpawningManager`. " +
                    "Initialize it with `CustomNPCSpawningManager.Initialize()`");
            }
        }
    }
}