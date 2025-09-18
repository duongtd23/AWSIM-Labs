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
    public class CustomSimManager
    {
        // singleton instance
        private static CustomSimManager _manager;

        private GameObject autowareEgoCar;
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
        private CustomSimManager()
        {
        }

        public static void Initialize(GameObject parent,
            GameObject taxi, GameObject hatchback,
            GameObject smallCar, GameObject truck, GameObject van,
            GameObject casualPedestrian, GameObject elegantPedestrian,
            LayerMask vehicleLM, LayerMask groundLM)
        {
            var manager = Manager();
            manager.npcTaxi = taxi;
            manager.npcHatchback = hatchback;
            manager.npcSmallCar = smallCar;
            manager.npcTruck = truck;
            manager.npcVan = van;
            manager._casualPedestrian = casualPedestrian;
            manager._elegantPedestrian = elegantPedestrian;
            manager.vehicleLayerMask = vehicleLM;
            manager.groundLayerMask = groundLM;
            manager.parentGameObject = parent;
            manager.egoStartMovingTime = -1;
            manager.egoEngagedTime = -1;
            manager.egoEngaged = false;
            manager.delayingMoveNPCs = new Dictionary<NPCVehicle, Tuple<int, NPCCar>>();
            manager.delayingSpawnNPCs = new List<NPCCar>();
            manager._delayingSpawnPedestrians = new List<NPCPedes>();
            manager.npcs = new List<NPCVehicle>();
            manager._pedestrians = new List<Tuple<NPCPedes, NPCPedestrian>>();

            try
            {
                SimulatorROS2Node.CreateSubscription<autoware_vehicle_msgs.msg.Engage>(
                    "/autoware/engage", msg =>
                    {
                        if (msg.Engage_)
                        {
                            Debug.Log("[AWAnalysis] Got /autoware/engage message: " + msg);
                            manager.egoEngaged = true;
                        }
                    });
            }
            catch (NullReferenceException e)
            {
                Debug.LogError("[AWAnalysis] Cannot create ROS subscriber /autoware/engage. " +
                    "Make sure Autoware has been started. Exception detail: " + e);
            }
        }

        public static void InitializeEgo(GameObject ego)
        {
            var manager = Manager();
            manager.autowareEgoCar = ego;
            NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();
            vehicleConfig.Acceleration = ConfigLoader.Config().NpcAcceleration;
            vehicleConfig.Deceleration = ConfigLoader.Config().NpcDeceleration;
            manager.npcVehicleSpawner = new NPCVehicleSpawner(manager.parentGameObject, 
                new GameObject[] { }, new TrafficLane[] { });
            
            manager.npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, 
                manager.vehicleLayerMask, manager.groundLayerMask, 10, ego);
        }

        public static CustomSimManager Manager()
        {
            if (_manager == null)
            {
                _manager = new CustomSimManager();
            }
            return _manager;
        }

        public static List<NPCVehicle> GetNPCs() => Manager().npcs;
        public static List<Tuple<NPCPedes, NPCPedestrian>> GetPedestrians() => Manager()._pedestrians;

        public static TrafficLane[] GetAllTrafficLanes()
        {
            if (Manager().allTrafficLanes == null)
            {
                var trafficLanesParent = GameObject.Find("TrafficLanes");
                if (trafficLanesParent != null)
                    Manager().allTrafficLanes = trafficLanesParent.GetComponentsInChildren<TrafficLane>();
            }

            return Manager().allTrafficLanes;
        }

        // this should be called every frame
        public void UpdateNPCs()
        {
            if (Manager() != null)
            {
                // check if Ego moved
                if (egoStartMovingTime <= 0 &&
                    CustomSimUtils.MagnitudeIgnoreYAxis(EgoSingletonInstance.AutowareEgoVehicle.Velocity) > 0.15f)
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

                UpdateSwerveNPC();
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
                        if (npcCar.Goal == null)
                        {
                            npcVehicleSimulator.Register(npcVehicle, 
                                CustomSimUtils.ParseLane(npcCar.InitialPosition.GetLane()), 
                                waypointIndex, 
                                npcCar.Config);
                        }
                        else
                        {
                            npcVehicleSimulator.Register(npcVehicle, waypointIndex,
                                npcCar.Goal,
                                npcCar.Config,
                                npcCar.VehicleType);
                        }
                        removeAfter.Add(npcVehicle);
                    }
                }
                else if (idelay is NPCDelayDistance delayDistance)
                {
                    // TODO: handle a curve lane (but straight)
                    if (EgoSingletonInstance.AutowareEgoVehicle.Velocity.magnitude > 0.1f &&
                        CustomSimUtils.LongitudeDistance(
                            EgoSingletonInstance.AutowareEgoVehicle.Position,
                            EgoSingletonInstance.AutowareEgoVehicle.Rotation * Vector3.forward,
                            npcVehicle.Position) <= delayDistance.Distance)
                    {
                        npcVehicleSimulator.Register(npcVehicle, waypointIndex, 
                            npcCar.Goal,
                            npcCar.Config,
                            npcCar.VehicleType);
                        Debug.Log($"[AWAnalysis] {npcCar.Name} started move.");
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
                        // if moving forward, and this is the last waypoint
                        if (!npcPedes.Backward && npcPedes.CurrentWaypointIndex == npcPedes.Waypoints.Count - 1)
                        {
                            // loop: turn backward
                            if (npcPedes.Config != null && npcPedes.Config.Loop)
                            {
                                npcPedes.Backward = true;
                                npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                                npcPedes.CurrentWaypointIndex--;
                                npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.CurrentWaypoint - npcPedes.LastPosition);
                                entry.Item2.SetRotation(npcPedes.LastRotation);
                            }
                            // reached the goal, do nothing
                            else
                            {
                                
                            }
                        }
                        // if moving backward, and this is the first waypoint
                        else if (npcPedes.Backward && npcPedes.CurrentWaypointIndex == 0)
                        {
                            if (npcPedes.Config != null && npcPedes.Config.Loop)
                            {
                                npcPedes.Backward = false;
                                npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                                npcPedes.CurrentWaypointIndex++;
                                npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.CurrentWaypoint - npcPedes.LastPosition);
                                entry.Item2.SetRotation(npcPedes.LastRotation);
                            }

                        }
                        // update rotation to match new waypoint
                        else
                        {
                            npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                            if (npcPedes.Backward)
                                npcPedes.CurrentWaypointIndex--;
                            else
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
                    float currentSpeed = EgoSingletonInstance.AutowareEgoVehicle.Velocity.magnitude;
                    if (currentSpeed > EgoSingletonInstance.CustomEgoSetting.EgoSettings.MaxVelocity)
                        currentSpeed = EgoSingletonInstance.CustomEgoSetting.EgoSettings.MaxVelocity;
                    
                    float desired_dx0 = currentSpeed * ConfigLoader.Config().TimeHeadWay;
                    float actual_dx0 = CustomSimUtils.DistanceIgnoreYAxis(EgoSingletonInstance.AutowareEgoVehicle.Position, cutoutNPC.Position);
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
                state.CustomConfig.AggressiveDrive &&
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
                    float desired_dx0 = Mathf.Min(EgoSingletonInstance.AutowareEgoVehicle.Velocity.magnitude, decelNPC.Velocity.magnitude) *
                                        ConfigLoader.Config().TimeHeadWay;
                    float actual_dx0 = CustomSimUtils.DistanceIgnoreYAxis(EgoSingletonInstance.AutowareEgoVehicle.Position, decelNPC.Position);
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

        private void UpdateSwerveNPC()
        {
            
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
                npc0.CustomConfig.AggressiveDrive &&
                npc0.CustomConfig.Deceleration >= 9.8f);
            if (results.Count >= 2)
            {
                Debug.LogError("Found more than one possible deceleration vehicle. Use the first one by default.");
            }
            return results.FirstOrDefault();
        }
        
        private NPCVehicle SwerveVehicle()
        {
            var results = GetNPCs().FindAll(npc =>
                npc.CustomConfig != null &&
                npc.CustomConfig.LateralWandering != null);
            if (results.Count >= 2)
            {
                Debug.LogError("Found more than one possible swerve vehicle. Use the first one by default.");
            }
            return results.FirstOrDefault();
        }
        
        private NPCVehicle UTurnVehicle()
        {
            var results = GetNPCs().FindAll(npc =>
                npc.CustomConfig != null &&
                npc.CustomConfig.UTurn != null);
            if (results.Count >= 2)
            {
                Debug.LogError("Found more than one possible U-Turn vehicle. Use the first one by default.");
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
                npcConfig,
                vehicleType);
            return npc;
        }

        // spawn an NPC and delay `delay.ActivateDelay` seconds to make it move
        public static NPCVehicle SpawnNPCAndDelayMovement(NPCCar npcCar)
        {
            if (npcCar.SpawnDelayOption == null || npcCar.SpawnDelayOption.ActionDelayed != DelayedAction.MOVING)
                throw new CustomSimException("[AWAnalysis]: Invalid NPCSpawnDelay parameter.");

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
        public static NPCVehicle GetSwerveVehicle() => Manager().SwerveVehicle();
        public static NPCVehicle GetUTurnVehicle() => Manager().UTurnVehicle();
        
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
                state.CustomConfig.AggressiveDrive &&
                state.CustomConfig.Deceleration >= 9.8f);
        public static NPCVehicleInternalState SwerveNPCInternalState() => 
            Manager().npcVehicleSimulator?.VehicleStates?.FirstOrDefault(state =>
                state.CustomConfig != null &&
                state.CustomConfig.LateralWandering != null);
        public static NPCVehicleInternalState UTurnNPCInternalState() => 
            Manager().npcVehicleSimulator?.VehicleStates?.FirstOrDefault(state =>
                state.CustomConfig != null &&
                state.CustomConfig.UTurn != null);
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
        
        public static Tuple<float,float> GetWheelBaseAndTurningWheelAngle(VehicleType vehicleType)
        {
            switch (vehicleType)
            {
                case VehicleType.TAXI:
                    return Tuple.Create(2.7f, 30f);
                case VehicleType.HATCHBACK:
                    return Tuple.Create(2.5f, 30f);
                case VehicleType.SMALL_CAR:
                    return Tuple.Create(2.5f, 30f);
                case VehicleType.TRUCK:
                    return Tuple.Create(4.5f, 35f);
                case VehicleType.VAN:
                    return Tuple.Create(2.7f, 30f);
            }
            throw new InvalidScriptException("Cannot detect the vehicle type `" + vehicleType + "`.");
        }

        private static void EnsureNonNullInstance(CustomSimManager instance)
        {
            if (instance == null)
            {
                Debug.Log("[AWAnalysis] Could not find an instance of `CustomNPCSpawningManager`.");
                throw new NullReferenceException("[NPCSim] Could not find an instance of `CustomNPCSpawningManager`. " +
                    "Initialize it with `CustomNPCSpawningManager.Initialize()`");
            }
        }

        /// <summary>
        /// reset delay to 0 for the $vehicle, i.e., making it move immediately
        /// </summary>
        /// <param name="vehicle"></param>
        public static void RemoveDelayFromNPC(NPCVehicle vehicle, float delay=0f)
        {
            var waypointId= _manager.delayingMoveNPCs[vehicle].Item1;
            var npcCar = _manager.delayingMoveNPCs[vehicle].Item2;
            // npcCar.SpawnDelayOption = NPCDelayTime.DelayMoveUntilEgoEngaged(delay);
            npcCar.SpawnDelayOption = NPCDelayTime.DelayMove(delay);
            _manager.delayingMoveNPCs[vehicle] = new Tuple<int, NPCCar>(waypointId, npcCar);
        }

        /// <summary>
        /// reset the lane of the $vehicle.
        /// Example use case: to reset the vehicle with a new virtual lane (e.g., constructed from a sequence of desired waypoints) 
        /// </summary>
        /// <param name="vehicle">must currently on $newLane</param>
        /// <param name="newLane"></param>
        public static void ResetLanePositionForNPC(NPCVehicle vehicle, TrafficLane newLane)
        {
            var npcCar = _manager.delayingMoveNPCs[vehicle].Item2;
            npcCar.InitialPosition = new LaneOffsetPosition(newLane.name, 0);
            _manager.delayingMoveNPCs[vehicle] = new Tuple<int, NPCCar>(1, npcCar);
        }

        public static void ResetMotionProfileForNPC(ref NPCVehicle vehicle,
            float targetSpeed, float acceleration, float deceleration,
            bool isSpeedDefined, bool isAccelerationDefined, bool isDecelerationDefined)
        {
            vehicle.CustomConfig = ResetMotionProfileForNPCConfig(vehicle.CustomConfig,
                targetSpeed, acceleration, deceleration,
                isSpeedDefined, isAccelerationDefined, isDecelerationDefined);
            
            if (_manager.delayingMoveNPCs[vehicle] != null)
            {
                var waypointId = _manager.delayingMoveNPCs[vehicle].Item1;
                var npcCar = _manager.delayingMoveNPCs[vehicle].Item2;
                npcCar.Config = ResetMotionProfileForNPCConfig(npcCar.Config,
                    targetSpeed, acceleration, deceleration,
                    isSpeedDefined, isAccelerationDefined, isDecelerationDefined);
                _manager.delayingMoveNPCs[vehicle] = new Tuple<int, NPCCar>(waypointId, npcCar);
            }        
        }
        
        public static NPCConfig ResetMotionProfileForNPCConfig(NPCConfig config,
            float targetSpeed, float acceleration, float deceleration,
            bool isSpeedDefined, bool isAccelerationDefined, bool isDecelerationDefined)
        {
            config ??= new NPCConfig();
            if (isSpeedDefined)
                config.TargetSpeed = targetSpeed;
            if (isAccelerationDefined)
                config.Acceleration = acceleration;
            if (isDecelerationDefined)
                config.Deceleration = deceleration;
            return config;
        }

        // despawn NPC immediately
        public static bool DespawnNPC(NPCVehicle vehicle)
        {
            var internalState = Manager().npcVehicleSimulator.VehicleStates.FirstOrDefault(state =>
                state.Vehicle == vehicle);
            if (internalState != null)
                internalState.ShouldDespawn = true;
            UnityEngine.Object.DestroyImmediate(vehicle.gameObject);
            Manager().npcs.Remove(vehicle);
            return true;
            // Debug.LogError($"[AWAnalysis] Could not find internal state of {vehicle}.");
        }
    }
}