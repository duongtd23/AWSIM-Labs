using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using autoware_adapi_v1_msgs.msg;
using autoware_adapi_v1_msgs.srv;
using autoware_vehicle_msgs.msg;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
using UnityEngine;
using AWSIM.AWAnalysis.CustomSim.DynamicCommand;
using AWSIM.TrafficSimulation;
using aw_monitor.srv;
using AWSIM_Script.Error;
using AWSIM.AWAnalysis.CustomSim.Objects;
using AWSIM.AWAnalysis.Monitor;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using geometry_msgs.msg;
using ROS2;
using ResponseStatus = aw_monitor.msg.ResponseStatus;
using Vector3 = UnityEngine.Vector3;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class DynamicSimControl : MonoBehaviour
    {
        public const string TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN = "/dynamic_control/vehicle/spawn";
        public const string TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE = "/dynamic_control/vehicle/follow_lane";

        public const string TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS =
            "/dynamic_control/vehicle/follow_waypoints";

        public const string TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING = "/dynamic_control/vehicle/removing";
        public const string TOPIC_DYNAMIC_CONTROL_AWSIM_SCRIPT = "/dynamic_control/script/awsim_script";
        public const string TOPIC_EGO_ESTIMATED_KINEMATICS = "/api/vehicle/kinematics";

        // service to check whether the spawning, follow lane, etc. actions sent before 
        // were successfully applied without any errors
        public const string SRV_DYNAMIC_CONTROL_VEHCILE_SPAWN = TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN + "_srv";

        public const string SRV_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE =
            TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE + "_srv";

        public const string SRV_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS =
            TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS + "_srv";

        public const string SRV_DYNAMIC_CONTROL_VEHICLE_REMOVING =
            TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING + "_srv";

        public const string SRV_DYNAMIC_CONTROL_AWSIM_SCRIPT =
            TOPIC_DYNAMIC_CONTROL_AWSIM_SCRIPT + "_srv";

        public const string LOCALIZATION_INITIALIZATION_SRV = "/api/localization/initialize";

        // queues of publisher messages sent from client (e.g., AWSIM-Script and Scenic)
        // Note that we cannot handle the requested action inside ROS subscription callbacks.
        // This is because the implementation needs to use some Unity functions that are only accessible from the main thread.
        // whereas, the callbacks are fired in the background thread (ROS spinning).
        private Queue<std_msgs.msg.String> _spawnReqQueue = new();
        private Queue<std_msgs.msg.String> _followLaneReqQueue = new();
        private Queue<std_msgs.msg.String> _followWaypointsReqQueue = new();
        private Queue<std_msgs.msg.String> _removeReqQueue = new();
        private Queue<std_msgs.msg.String> _awsimScriptReqQueue = new();

        // saving the map (requests |-> responses), where
        // requests are (in form of json string) published msg from clients for making actions (e.g., spawning)
        // responses are (instance of DynamicControl_Response) are the processed results.
        // Note that it is impossible to implement a service server instead, i.e.,
        // blocking until finishing processing actions.
        // This is because the action implementation must be done in the main thread.
        Dictionary<string, DynamicControl_Response> _spawnReqResDict = new();
        Dictionary<string, DynamicControl_Response> _followLaneReqResDict = new();
        Dictionary<string, DynamicControl_Response> _followWaypointsReqResDict = new();
        Dictionary<string, DynamicControl_Response> _removeReqResDict = new();
        Dictionary<string, DynamicControl_Response> _awsimScriptReqResDict = new();

        public void Start()
        {
            var qos = new QoSSettings
            {
                ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                Depth = 1,
            }.GetQoSProfile();

            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN,
                msg => { _spawnReqQueue.Enqueue(msg); },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE,
                msg => { _followLaneReqQueue.Enqueue(msg); },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS,
                msg => { _followWaypointsReqQueue.Enqueue(msg); },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING,
                msg => { _removeReqQueue.Enqueue(msg); },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_AWSIM_SCRIPT,
                msg => { _awsimScriptReqQueue.Enqueue(msg); },
                qos);

            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHCILE_SPAWN,
                msg =>
                    _spawnReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE,
                msg =>
                    _followLaneReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS,
                msg =>
                    _followWaypointsReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHICLE_REMOVING,
                msg =>
                    _removeReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_AWSIM_SCRIPT,
                msg =>
                    _awsimScriptReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));
        }

        private DynamicControl_Response UNPROCESSED_REQ()
        {
            return new DynamicControl_Response()
            {
                Status = new ResponseStatus()
                {
                    Code = 2,
                    Success = false,
                    Message = "Unprocessed for invalid request."
                }
            };
        }

        private DynamicControl_Response INVALID_REQ(Exception e)
        {
            return new DynamicControl_Response()
            {
                Status = new ResponseStatus()
                {
                    Code = 3,
                    Success = false,
                    Message = e.Message
                }
            };
        }

        public void Update()
        {
            while (_spawnReqQueue.Count > 0)
            {
                var req = _spawnReqQueue.Dequeue();
                DynamicControl_Response response = null;

                try
                {
                    var command = JsonUtility.FromJson<DynamicSpawnCommand>(req.Data);
                    Debug.Log($"Parsed command: {command}");
                    response = HandleSpawnAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _spawnReqResDict[req.Data] = response;
                }
            }

            while (_followLaneReqQueue.Count > 0)
            {
                var req = _followLaneReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<DynamicFollowLaneCommand>(req.Data);
                    Debug.Log($"Parsed command: {command}");
                    response = HandleFollowLaneAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _followLaneReqResDict[req.Data] = response;
                }
            }

            while (_followWaypointsReqQueue.Count > 0)
            {
                var req = _followWaypointsReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<DynamicFollowWaypointCommand>(req.Data);
                    Debug.Log($"Parsed command: {command}");
                    response = HandleFollowWaypointsAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _followWaypointsReqResDict[req.Data] = response;
                }
            }

            while (_removeReqQueue.Count > 0)
            {
                var req = _removeReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<DynamicRemoveCommand>(req.Data);
                    Debug.Log($"Parsed command: {command}");
                    response = HandleRemoveAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _removeReqResDict[req.Data] = response;
                }
            }

            while (_awsimScriptReqQueue.Count > 0)
            {
                var req = _awsimScriptReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<DynamicAWSIMScriptCommand>(req.Data);
                    // Debug.Log($"Parsed command: {command}");
                    response = HandleAWSIMScriptScenario(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _awsimScriptReqResDict[req.Data] = response;
                }
            }
        }

        private DynamicControl_Response HandleSpawnAction(DynamicSpawnCommand command)
        {
            var position = ROS2Utility.RosMGRSToUnityPosition(command.position);
            var lane = CustomSimUtils.LaneAtPosition(position, out int waypointId, out float laneOffset,
                tolerance: 0.5f);
            if (lane == null)
            {
                Debug.LogError($"Cannot find lane for the spawning position {position}");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"Cannot find lane for the spawning position {position}.",
                        Success = false
                    }
                };
            }

            var spawnPosition = new LaneOffsetPosition(lane.name, laneOffset);

            // construct configuration
            NPCCar npc = new NPCCar(ScenarioParser.ParseVehicleType(command.body_style), spawnPosition);
            npc.Name = command.name;
            npc.SpawnDelayOption = NPCDelayTime.DelayMoveUntilEgoEngaged(float.MaxValue);
            CustomSimManager.SpawnNPCAndDelayMovement(npc);
            Debug.Log(
                $"[AWAnalysis] spawned NPC {command.name} at position {position}, lane {lane.name}, offset {laneOffset}");

            return new DynamicControl_Response
            {
                Status = new ResponseStatus
                {
                    Code = 0,
                    Message = "Success",
                    Success = true
                }
            };
        }

        private DynamicControl_Response HandleFollowLaneAction(DynamicFollowLaneCommand command)
        {
            var targetNPC = CustomSimManager.GetNPCs().Find(npc => npc.ScriptName == command.target);
            if (targetNPC == null)
            {
                Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"NPC {command.target} not found.",
                        Success = false
                    }
                };
            }

            CustomSimManager.ResetMotionProfileForNPC(ref targetNPC,
                command.speed, command.acceleration, command.deceleration,
                command.is_speed_defined, command.is_acceleration_defined, command.is_deceleration_defined);

            CustomSimManager.RemoveDelayFromNPC(targetNPC);
            Debug.Log($"[AWAnalysis] Sent follow lane command to NPC {command.target}");
            return new DynamicControl_Response
            {
                Status = new ResponseStatus
                {
                    Code = 0,
                    Message = "Success",
                    Success = true
                }
            };
        }

        private DynamicControl_Response HandleFollowWaypointsAction(DynamicFollowWaypointCommand command)
        {
            var targetNPC = CustomSimManager.GetNPCs().Find(npc => npc.ScriptName == command.target);
            if (targetNPC == null)
            {
                Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"NPC {command.target} not found.",
                        Success = false
                    }
                };
            }

            List<Vector3> waypoints = new List<Vector3>();
            foreach (var point in command.waypoints)
            {
                waypoints.Add(ROS2Utility.RosMGRSToUnityPosition(point));
                Debug.Log($"[AWAnalysis] Waypoint: {waypoints.Last()}");
            }
            // TODO: handle case waypoint.z = 0

            // construct a virtual traffic lane
            // find the lane on which the last waypoint located
            var lane = CustomSimUtils.LaneAtPosition(waypoints.Last(), out int waypointId, out float laneOffset,
                tolerance: 0.5f);
            TrafficLane virtualLane = Instantiate(lane);

            // construct the waypoints for the virtual lane.
            List<Vector3> virtualLaneWaypoints = new List<Vector3>();
            // The current position of NPC should be inserted as the first waypoint
            // if (CustomSimUtils.DistanceIgnoreYAxis(targetNPC.Position, waypoints[0]) > 1)
            //     virtualLaneWaypoints.Add(targetNPC.Position);

            // add specified waypoints
            virtualLaneWaypoints.AddRange(waypoints);
            // add $lane's waypoints after the last specified waypoint (in order to connect the next lane(s) of $lane) 
            for (int i = waypointId + 1; i < lane.Waypoints.Length; i++)
                virtualLaneWaypoints.Add(lane.Waypoints[i]);
            virtualLane.UpdateWaypoints(virtualLaneWaypoints.ToArray());

            // reset virtual lane's previous 
            virtualLane.ResetPrevLanes(new List<TrafficLane>());
            virtualLane.ResetNextLanes(lane.NextLanes);

            // config the virtual lane as the NPC route (without goal), and let it move
            CustomSimManager.ResetLanePositionForNPC(targetNPC, virtualLane);
            CustomSimManager.ResetMotionProfileForNPC(ref targetNPC,
                command.speed, command.acceleration, command.deceleration,
                command.is_speed_defined, command.is_acceleration_defined, command.is_deceleration_defined);
            CustomSimManager.RemoveDelayFromNPC(targetNPC);

            return new DynamicControl_Response
            {
                Status = new ResponseStatus
                {
                    Code = 0,
                    Message = "Success",
                    Success = true
                }
            };
        }

        private DynamicControl_Response HandleRemoveAction(DynamicRemoveCommand command)
        {
            List<String> unsuccessfulTargets = new List<String>();
            
            if (string.IsNullOrEmpty(command.target))
            {
                // remove all NPCs
                int noNPCs = CustomSimManager.GetNPCs().Count;
                for (int i = noNPCs - 1; i >= 0; i--)
                {
                    var npc = CustomSimManager.GetNPCs()[i];
                    if (!RemoveSingleNPC(npc))
                        unsuccessfulTargets.Add(npc.ScriptName);
                }
            }
            else
            {
                var targetNPC = CustomSimManager.GetNPCs().Find(npc => npc.ScriptName == command.target);
                if (targetNPC == null)
                {
                    Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                    return new DynamicControl_Response
                    {
                        Status = new ResponseStatus
                        {
                            Code = 1,
                            Message = $"NPC {command.target} not found.",
                            Success = false
                        }
                    };
                }
                if (!RemoveSingleNPC(targetNPC))
                    unsuccessfulTargets.Add(command.target);
            }
            
            if (unsuccessfulTargets.Count > 0)
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"Could not despawn NPC(s) {string.Join(", ", unsuccessfulTargets)}.",
                        Success = false
                    }
                };
            return new DynamicControl_Response
            {
                Status = new ResponseStatus
                {
                    Code = 0,
                    Message = "",
                    Success = true
                }
            };
        }

        private bool RemoveSingleNPC(NPCVehicle target)
        {
            return CustomSimManager.DespawnNPC(target);
        }

        private DynamicControl_Response HandleAWSIMScriptScenario(DynamicAWSIMScriptCommand command)
        {
            var scriptFile = command.file;
            Debug.Log("Loading input script: " + scriptFile);
            try
            {
                Simulation simulation = new ScriptParser().ParseScriptFromFile(scriptFile);
                var ok = ResetEgoSetting(simulation, 
                    out PoseWithCovarianceStamped poseMsg, 
                    out PoseStamped goalMsg);
                
                string message = "No Ego specification in the input script";
                if (ok)
                {
                    var poseAndGoal = InitPoseAndGoalObject.FromRosPoseAndGoal(poseMsg.Pose, goalMsg.Pose);
                    message = JsonUtility.ToJson(poseAndGoal);
                }
                
                PreProcessingSimulation(ref simulation);
                ExecuteSimulation(simulation);
                PostProcessingSimulation(ref simulation);
                
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 0,
                        Message = message,
                        Success = true
                    }
                };
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = e.Message,
                        Success = false
                    }
                };
            }
        }

        private bool ResetEgoSetting(Simulation simulation,
                out geometry_msgs.msg.PoseWithCovarianceStamped poseMsg,
                out geometry_msgs.msg.PoseStamped goalMsg)
        {
            if (simulation.Ego != null)
            {
                var customEgoSetting = new CustomEgoSetting(simulation.Ego);
                EgoSingletonInstance.SetCustomEgoSetting(customEgoSetting);
                EgoSingletonInstance.CustomEgoSetting.SetInitPose(EgoSingletonInstance.CustomEgoSetting.LastInitialPose);
                
                // reset our tracked state
                ExecutionStateTracker.ResetState();

                poseMsg = EgoSingletonInstance.CustomEgoSetting.LastInitialPose;
                goalMsg = EgoSingletonInstance.CustomEgoSetting.LastGoal;
                return true;
            }
            poseMsg = null;
            goalMsg = null;
            return false;
        }

        private void ExecuteSimulation(Simulation simulation)
        {
            foreach (NPCCar npcCar in simulation.NPCs)
            {
                CustomSimManager.SpawnNPC(npcCar);
            }

            foreach (var npcPedes in simulation.Pedestrians)
            {
                CustomSimManager.SpawnPedestrian(npcPedes);
            }
        }

        private void PreProcessingSimulation(ref Simulation simulation)
        {
            for (int i = 0; i < simulation.NPCs.Count; i++)
            {
                var npc = simulation.NPCs[i];
                if (npc.HasConfig() &&
                    npc.Config.HasALaneChange())
                {
                    if (npc.Config.LaneChange is CutInLaneChange)
                    {
                        PreProcessingCutIn(ref npc, ref simulation);
                        TrafficLane cutinLane = CustomSimUtils.ParseLane(npc.Config.LaneChange.SourceLane);
                        var cutinPoint = CustomSimUtils.CalculatePosition(
                            cutinLane, npc.Config.LaneChange.ChangeOffset, out int _);
                        PublishMetadata("cutin_point", cutinPoint);
                    }
                }
            }
        }

        private void PublishMetadata(string key, Vector3 unityPoint)
        {
            var rosPoint = new Vector3Object(ROS2Utility.UnityToRosMGRS(unityPoint));
            GroundTruthInfoPublisher gtInfoPublisher = FindObjectOfType<AWAnalysis>().GtInfoPublisher;
            if (gtInfoPublisher != null)
            {
                gtInfoPublisher.SetMetadataAndPublish("{\"" + key + "\": " + JsonUtility.ToJson(rosPoint) + "}");
            }
        }

        // compute initial position
        private void PreProcessingCutIn(ref NPCCar npc, ref Simulation simulation)
        {
            EgoDetailObject egoDetailObject = EgoSingletonInstance.GetFixedEgoDetailInfo();
            NPCDetailObject npcDetailObject = CustomSimManager.GetNPCCarInfo(npc.VehicleType);
            var cutInLaneChange = npc.Config.LaneChange as CutInLaneChange;
            float desiredDX = cutInLaneChange.Dx;
            float timeNPCTravelBeforeCutin = ConfigLoader.Config().TimeNPCTravelBeforeCutin;
            float desiredSpeed = npc.Config.GetDesiredSpeed(cutInLaneChange.SourceLane);
            float acceleration = npc.Config.Acceleration;
            float egoSpeed = simulation.Ego.MaxVelocity;
            float npcTotalTravelDisBeforeCutin = 0.5f * desiredSpeed * desiredSpeed / acceleration +
                                                 timeNPCTravelBeforeCutin * desiredSpeed;
            float d0 = (desiredSpeed / acceleration + timeNPCTravelBeforeCutin) * egoSpeed -
                       npcTotalTravelDisBeforeCutin +
                       desiredDX +
                       (float)(egoDetailObject.extents.z + egoDetailObject.center.z) +
                       (float)(npcDetailObject.extents.z - npcDetailObject.center.z);
            // d0 is the distance from ego to NPC such that
            // if NPC starts moving when the distance between ego and NPC reaches this value,
            // the desired Dx will be satisfied
            Debug.Log($"Computed D0: {d0}");

            float egoAcceleration = ConfigLoader.Config().EgoNormalAcceleration;
            float distanceForEgoReachDesiredSpeed = 0.5f * egoSpeed * egoSpeed / egoAcceleration;
            float timeEgoTravelConstSpeed = ConfigLoader.Config().TimeEgoTravelConstSpeed;
            float distanceEgoTravelConstSpeed = timeEgoTravelConstSpeed * egoSpeed;
            Vector3 egoInitPosition = CustomSimUtils.CalculatePosition(simulation.Ego.InitialPosition);
            TrafficLane sourceLane = CustomSimUtils.ParseLane(cutInLaneChange.SourceLane);
            float egoOffsetProjectedOnSourceLane = CustomSimUtils.LongitudeDistance(
                sourceLane.Waypoints[0],
                sourceLane.Waypoints[1] - sourceLane.Waypoints[0],
                egoInitPosition);

            if (Vector3.Dot(sourceLane.Waypoints[1] - sourceLane.Waypoints[0],
                    egoInitPosition - sourceLane.Waypoints[0]) < 0)
                egoOffsetProjectedOnSourceLane = -egoOffsetProjectedOnSourceLane;

            float initialPosOffset = distanceForEgoReachDesiredSpeed +
                                     distanceEgoTravelConstSpeed +
                                     d0 +
                                     egoOffsetProjectedOnSourceLane;

            Debug.Log($"NPC initial position offset: {initialPosOffset}");

            if (sourceLane.TotalLength() <= initialPosOffset)
            {
                // TODO: handle the case when lane length is not sufficient
                throw new InvalidScriptException($"{cutInLaneChange.SourceLane}'s length is not sufficient.");
            }

            npc.InitialPosition = new LaneOffsetPosition(cutInLaneChange.SourceLane, initialPosOffset);
            cutInLaneChange.ChangeOffset = initialPosOffset + npcTotalTravelDisBeforeCutin +
                                           (float)(npcDetailObject.extents.z + npcDetailObject.center.z -
                                                   0.3f); // TODO: remove hard code 0.3f
            npc.SpawnDelayOption = NPCDelayDistance.DelayMove(d0);
        }

        // mainly compute NPCDelayDistance for $npc movement and Swerve, U-Turn behaviors
        private void PostProcessingSimulation(ref Simulation simulation)
        {
            for (int j = 0; j < simulation.NPCs.Count; j++)
            {
                var npc = simulation.NPCs[j];
                if (npc.HasConfig() &&
                    npc.Config.LateralWandering != null &&
                    !Mathf.Approximately(npc.Config.LateralWandering.Dx, LateralWandering.DUMMY_DX) &&
                    npc.HasDelayOption() && npc.SpawnDelayOption.ActionDelayed == DelayedAction.MOVING &&
                    npc.SpawnDelayOption is NPCDelayTime delayTime &&
                    Mathf.Approximately(delayTime.DelayAmount, NPCDelayTime.DUMMY_DELAY_AMOUNT))
                {
                    PostProcessingSwerve(ref npc);
                    TrafficLane swerveLane = CustomSimUtils.ParseLane(npc.Config.LateralWandering.SourceLane);
                    var swervePoint = CustomSimUtils.CalculatePosition(
                        swerveLane, npc.Config.LateralWandering.WanderOffset, out int _);
                    PublishMetadata("swerve_point", swervePoint);
                }

                else if (npc.HasConfig() &&
                         npc.Config.UTurn != null &&
                         !Mathf.Approximately(npc.Config.UTurn.Dx, UTurn.DUMMY_DX) &&
                         npc.HasDelayOption() && npc.SpawnDelayOption.ActionDelayed == DelayedAction.MOVING &&
                         npc.SpawnDelayOption is NPCDelayTime delayTime2 &&
                         Mathf.Approximately(delayTime2.DelayAmount, NPCDelayTime.DUMMY_DELAY_AMOUNT))
                {
                    PostProcessingUTurn(ref npc);
                    TrafficLane uturnLane = CustomSimUtils.ParseLane(npc.Config.UTurn.SourceLane);
                    var uturnPoint = CustomSimUtils.CalculatePosition(
                        uturnLane, npc.Config.UTurn.UTurnOffset, out int _);
                    PublishMetadata("uturn_point", uturnPoint);
                }
            }
        }

        /// <summary>
        /// mainly to compute the $distancedelay.
        /// When the distance between Ego and NPC falls below $distancedelay, NPC will start moving
        /// </summary>
        /// <param name="npc"></param>
        private void PostProcessingSwerve(ref NPCCar npc)
        {
            EgoDetailObject egoDetailObject = EgoSingletonInstance.GetFixedEgoDetailInfo();
            NPCDetailObject npcDetailObject = CustomSimManager.GetNPCCarInfo(npc.VehicleType);

            string sourceLaneStr = npc.Config.LateralWandering.SourceLane;
            float sourceLaneSpeed = npc.RouteAndSpeeds
                .First(entry => entry.Item1 == sourceLaneStr)
                .Item2;

            float distance2SwerveWp = DistanceAndTimeToWp(npc, npcDetailObject,
                sourceLaneStr,
                npc.Config.LateralWandering.WanderOffset, out float time2SwerveWp);
            // Debug.Log($"[AWAnalysis] distance2SwerveWp: {distance2SwerveWp}");

            float distancedelay = npc.Config.LateralWandering.Dx +
                                  (float)(egoDetailObject.RootToFront() + npcDetailObject.RootToFront()) +
                                  distance2SwerveWp +
                                  time2SwerveWp * EgoSingletonInstance.DesiredMaxVelocity();
            npc.SpawnDelayOption = NPCDelayDistance.DelayMove(distancedelay);
            Debug.Log($"[AWAnalysis] distance delay is: {distancedelay}");
        }

        private void PostProcessingUTurn(ref NPCCar npc)
        {
            EgoDetailObject egoDetailObject = EgoSingletonInstance.GetFixedEgoDetailInfo();
            NPCDetailObject npcDetailObject = CustomSimManager.GetNPCCarInfo(npc.VehicleType);

            string sourceLaneStr = npc.Config.UTurn.SourceLane;
            float sourceLaneSpeed = npc.RouteAndSpeeds
                .First(entry => entry.Item1 == sourceLaneStr)
                .Item2;

            float distance2UTurnWp = DistanceAndTimeToWp(npc, npcDetailObject,
                sourceLaneStr,
                npc.Config.UTurn.UTurnOffset, out float time2UTurnWp);

            float distancedelay = npc.Config.UTurn.Dx +
                                  (float)(egoDetailObject.RootToFront() + npcDetailObject.RootToFront()) +
                                  distance2UTurnWp +
                                  time2UTurnWp * EgoSingletonInstance.DesiredMaxVelocity();
            npc.SpawnDelayOption = NPCDelayDistance.DelayMove(distancedelay);
            Debug.Log($"[AWAnalysis] distance delay is: {distancedelay}");
        }


        /// <summary>
        /// compute distance and time required to reach the $waypoint from the spawning position
        /// with predefined acceleration in the input script
        /// </summary>
        /// <param name="npc"></param>
        /// <param name="wpLaneStr"> the lane that $waypoint belongs to</param>
        /// <param name="waypointOffset">offset of the waypoint where swerve/u-turn starts</param>
        /// <param name="timeRequired">time required for $npc reach $waypoint</param>
        /// <returns></returns>
        private float DistanceAndTimeToWp(NPCCar npc, NPCDetailObject npcDetailObject,
            string wpLaneStr, float waypointOffset, out float timeRequired)
        {
            var acceleration = Mathf.Approximately(npc.Config.Acceleration, NPCConfig.DUMMY_ACCELERATION)
                ? NPCVehicleConfig.Default().Acceleration
                : npc.Config.Acceleration;

            // distance from the spawning point to the waypoint where swerve starts
            float distance2Wp = 0;
            // 
            // Suppose that $npc goes with constant speeds
            timeRequired = 0;
            int i = 0;
            for (; i < npc.RouteAndSpeeds.Count; i++)
            {
                string laneStr = npc.RouteAndSpeeds[i].Item1;
                if (laneStr == wpLaneStr)
                    break;
                var lane = CustomSimUtils.ParseLane(laneStr);
                if (i == 0)
                {
                    var laneDis = lane.TotalLength() - npc.InitialPosition.GetOffset();
                    distance2Wp += laneDis;
                    float speedUpTime = npc.RouteAndSpeeds[i].Item2 / acceleration;
                    float speedUpDistance = 0.5f * acceleration * speedUpTime * speedUpTime;
                    timeRequired += speedUpTime + (laneDis - speedUpDistance) / npc.RouteAndSpeeds[i].Item2;
                }
                else
                {
                    var laneDis = lane.TotalLength();
                    distance2Wp += laneDis;
                    timeRequired += laneDis / npc.RouteAndSpeeds[i].Item2;
                }
            }

            if (i == 0)
            {
                distance2Wp = waypointOffset -
                              npc.InitialPosition.GetOffset() -
                              (float)npcDetailObject.RootToFront();
                float speedUpTime = npc.RouteAndSpeeds[0].Item2 / acceleration;
                float speedUpDistance = 0.5f * acceleration * speedUpTime * speedUpTime;
                timeRequired = speedUpTime;
                // this should always happen
                if (speedUpDistance < distance2Wp)
                    timeRequired += (distance2Wp - speedUpDistance) / npc.RouteAndSpeeds[i].Item2;
            }
            else
            {
                distance2Wp += waypointOffset;
                timeRequired += waypointOffset / npc.RouteAndSpeeds[i].Item2;
            }

            return distance2Wp;
        }
    }
}
     