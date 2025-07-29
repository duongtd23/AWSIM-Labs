using System;
using System.Collections.Generic;
using System.Linq;
using aw_monitor.msg;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
using UnityEngine;
using AWSIM.AWAnalysis.CustomSim.DynamicCommand;
using AWSIM.TrafficSimulation;
using aw_monitor.srv;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using ROS2;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class DynamicSimControl : MonoBehaviour
    {
        public const string TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN = "/dynamic_control/vehicle/spawn";
        public const string TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE = "/dynamic_control/vehicle/follow_lane";
        public const string TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS = "/dynamic_control/vehicle/follow_waypoints";
        public const string TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING = "/dynamic_control/vehicle/removing";
        
        // service to check whether the spawning, follow lane, etc. actions sent before 
        // were successfully applied without any errors
        public const string SRV_DYNAMIC_CONTROL_VEHCILE_SPAWN = TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN + "_srv";
        public const string SRV_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE =
            TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE + "_srv";
        public const string SRV_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS =
            TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS + "_srv";
        public const string SRV_DYNAMIC_CONTROL_VEHICLE_REMOVING =
            TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING  + "_srv";
        
        // queues of publisher messages sent from client (e.g., Scenic)
        // Note that we cannot handle the requested action inside ROS subscription callbacks.
        // This is because the implementation needs to use some Unity functions that are only accessible from the main thread.
        // whereas, the callbacks are fired in the background thread (ROS spinning).
        private Queue<std_msgs.msg.String> _spawnReqQueue = new ();
        private Queue<std_msgs.msg.String> _followLaneReqQueue = new ();
        private Queue<std_msgs.msg.String> _followWaypointsReqQueue = new ();
        private Queue<std_msgs.msg.String> _removeReqQueue = new ();
        
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
                msg =>
                {
                    _spawnReqQueue.Enqueue(msg);
                },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE,
                msg =>
                {
                    _followLaneReqQueue.Enqueue(msg);
                },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS,
                msg =>
                {
                    _followWaypointsReqQueue.Enqueue(msg);
                },
                qos);
            SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING,
                msg =>
                {
                    _removeReqQueue.Enqueue(msg);
                },
                qos);
            
            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHCILE_SPAWN, HandleSpawnRequest);
            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE, HandleFollowLaneRequest);
            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS, HandleFollowWaypointsRequest);
            SimulatorROS2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHICLE_REMOVING, HandleRemoveRequest);
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
        }
        
        private DynamicControl_Response HandleSpawnRequest(DynamicControl_Request msg)
        {
            return _spawnReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ());
        }
        
        private DynamicControl_Response HandleFollowLaneRequest(DynamicControl_Request msg)
        {
            return _followLaneReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ());
        }

        private DynamicControl_Response HandleFollowWaypointsRequest(DynamicControl_Request msg)
        {
            return _followWaypointsReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ());
        }

        private DynamicControl_Response HandleRemoveRequest(DynamicControl_Request msg)
        {
            return _removeReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ());
        }
        
        private DynamicControl_Response HandleSpawnAction(DynamicSpawnCommand command)
        {
            var position = ROS2Utility.RosMGRSToUnityPosition(command.position);
            var lane = CustomSimUtils.LaneAtPosition(position, out int waypointId, out float laneOffset, tolerance:0.5f);
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
            Debug.Log($"[AWAnalysis] spawned NPC {command.name} at position {position}, lane {lane.name}, offset {laneOffset}");
            
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
            var lane = CustomSimUtils.LaneAtPosition(waypoints.Last(), out int waypointId, out float laneOffset, tolerance:0.5f);
            TrafficLane virtualLane = Instantiate(lane);
            
            // construct the waypoints for the virtual lane.
            List<Vector3> virtualLaneWaypoints = new List<Vector3>();
            // The current position of NPC should be inserted as the first waypoint
            // if (CustomSimUtils.DistanceIgnoreYAxis(targetNPC.Position, waypoints[0]) > 1)
            //     virtualLaneWaypoints.Add(targetNPC.Position);
            
            // add specified waypoints
            virtualLaneWaypoints.AddRange(waypoints);
            // add $lane's waypoints after the last specified waypoint (in order to connect the next lane(s) of $lane) 
            for (int i = waypointId+1; i < lane.Waypoints.Length; i++)
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

            if (CustomSimManager.DespawnNPC(targetNPC))
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 0,
                        Message = "",
                        Success = true
                    }
                };
            return new DynamicControl_Response
            {
                Status = new ResponseStatus
                {
                    Code = 1,
                    Message = $"Could not despawn NPC {command.target}.",
                    Success = false
                }
            };
        }
    }
}