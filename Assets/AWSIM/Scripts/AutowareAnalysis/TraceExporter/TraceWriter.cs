using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using AWSIM.AWAnalysis.CustomSim;
using autoware_adapi_v1_msgs.msg;
using autoware_perception_msgs.msg;
using autoware_vehicle_msgs.msg;
using AWSIM_Script.Object;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using AWSIM.TrafficSimulation;
using tier4_perception_msgs.msg;
using ROS2;
using awTrajectoryPoint = autoware_planning_msgs.msg.TrajectoryPoint;
using Shape = autoware_perception_msgs.msg.Shape;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public abstract class TraceWriter
    {
        protected string _filePath;
        protected GameObject _autowareEgoCar;
        protected readonly Vehicle _egoVehicle;
        protected readonly Camera _sensorCamera;
        protected readonly TraceCaptureConfig _config;
        protected readonly PerceptionMode _perceptionMode;
        protected readonly float _maxDistanceVisibleOnCamera;
        
        // inner use
        protected TraceCaptureState _state;
        protected float _timeNow;
        protected double _startTime;
        protected TraceObject _traceObject;
        // a detected object message with timeStamp > 10 steps (10*25=250ms) behind
        // the current frame will be discarded (with log error)
        protected const int MAX_LAG_FIXED_STEPS = 10;
        
        // ROS time at start up
        // protected double _rosTimeAtStart;
        // time when autonomous operation mode becomes ready
        protected float _autoOpModeReadyTime = -1f;
        
        protected Queue<Tuple<double, PredictedObject[]>> _objectDetectedMsgs;
        protected Queue<Tuple<double, DetectedObjectWithFeature[]>> _cameraObjectDetectedMsgs;
        protected Queue<Tuple<double, awTrajectoryPoint[]>> _planTrajectoryMsgs;
        
        ISubscription<OperationModeState> opModeSubscriber;
        ISubscription<RouteState> routeStateSubscriber;
        ISubscription<LocalizationInitializationState> localizationInitStateSubscriber;
        
        public TraceWriter(string filePath, Camera sensorCamera, 
            PerceptionMode perceptionMode, TraceCaptureConfig config)
        {
            _filePath = filePath;
            _autowareEgoCar = EgoSingletonInstance.AutowareEgoCarGameObject;
            _egoVehicle = EgoSingletonInstance.AutowareEgoCarVehicle;
            _sensorCamera = sensorCamera;
            _config = config;
            _perceptionMode = perceptionMode;
            InitializeTraceObj();
            _traceObject.states = new List<StateObject>();
            _objectDetectedMsgs = new Queue<Tuple<double, PredictedObject[]>>();
            _cameraObjectDetectedMsgs = new Queue<Tuple<double, DetectedObjectWithFeature[]>>();
            _planTrajectoryMsgs = new Queue<Tuple<double, awTrajectoryPoint[]>>();
            _maxDistanceVisibleOnCamera = ConfigLoader.Config().MaxDistanceVisibleonCamera;
        }

        protected void InitializeTraceObj()
        {
            _traceObject = new TraceObject
            {
                fixedTimestep = (int)(Time.fixedDeltaTime * 1000),
                camera_screen_height = _sensorCamera.pixelHeight,
                camera_screen_width = _sensorCamera.pixelWidth
            };

            if (CustomNPCSpawningManager.GetCutInVehicle() != null)
                _traceObject.other = new CutInInfoObject();
            else if (CustomNPCSpawningManager.GetCutOutVehicle() != null)
                _traceObject.other = new CutOutInfoObject();
            else if (CustomNPCSpawningManager.GetDecelerationVehicle() != null)
                _traceObject.other = new DecelerationInfoObject();
        }
        
        public void Start()
        {
            _state = TraceCaptureState.INITIALIZED;
            // difference between ROS time and Unity time
            // updated: since we use Unity time source, this is no longer needed
            // var rosTime = SimulatorROS2Node.GetCurrentRosTime();
            // _rosTimeAtStart = rosTime.Sec + rosTime.Nanosec / Math.Pow(10, 9);
            switch (_config.TraceCaptureFrom)
            {
                case CaptureStartingTime.AW_AUTO_MODE_READY:
                    opModeSubscriber = SimulatorROS2Node.CreateSubscription<OperationModeState>(
                        TopicName.TOPIC_API_OPERATION_MODE_STATE, msg =>
                        {
                            if (msg.Is_autonomous_mode_available && _autoOpModeReadyTime < 0)
                            {
                                _autoOpModeReadyTime = _timeNow;
                                _state = TraceCaptureState.AUTO_MODE_READY;
                                SubscribeRosEvents();
                            }
                        });
                    break;
                case CaptureStartingTime.AW_LOCALIZATION_INITIALIZED:
                    try
                    {
                        localizationInitStateSubscriber = SimulatorROS2Node.CreateSubscription<LocalizationInitializationState>(
                        TopicName.TOPIC_LOCALIZATION_INITIALIZATION_STATE, msg =>
                        {
                            if (msg.State == LocalizationInitializationState.INITIALIZED)
                            {
                                _state = TraceCaptureState.READY_TO_CAPTURE;
                                SubscribeRosEvents();
                            }
                        });
                    }
                    catch (NullReferenceException e)
                    {
                        Debug.LogError("[AWAnalysis] Cannot create ROS subscriber. " +
                            "Make sure Autoware has been started. Exception detail: " + e);
                    }
                    break;
                case CaptureStartingTime.AWSIM_STARTED:
                    _state = TraceCaptureState.READY_TO_CAPTURE;
                    SubscribeRosEvents();
                    break;
            }
        }

        public void Update()
        {
            _timeNow = Time.fixedTime;
            switch (_state)
            {
                case TraceCaptureState.INITIALIZED: 
                case TraceCaptureState.TRACE_WRITTEN:
                    break;
                case TraceCaptureState.AUTO_MODE_READY:
                    if (_timeNow >= _autoOpModeReadyTime + ConfigLoader.Config().DelaySendingEngageCmd)
                    {
                        _state = TraceCaptureState.READY_TO_CAPTURE;
                        // sending engage command
                        var engageMsg = new Engage();
                        engageMsg.Engage_ = true;
                        SimulatorROS2Node.CreatePublisher<Engage>(
                            TopicName.TOPIC_AUTOWARE_ENGAGE).Publish(engageMsg);
                    }
                    break;
                case TraceCaptureState.READY_TO_CAPTURE:
                    _startTime = _timeNow;
                    _state = TraceCaptureState.TRACE_CAPTURING;
                    Debug.Log("[AWAnalysis] Start capturing trace");
                    break;
                
                case TraceCaptureState.TRACE_CAPTURING:
                    // if saving-timeout is reached
                    if (_config.SavingTimeout != Simulation.DUMMY_SAVING_TIMEOUT &&
                        _timeNow > _config.SavingTimeout + _startTime)
                    {
                        _traceObject.comment = "Timeout reached before Ego arrives goal.";
                        _state = TraceCaptureState.EGO_GOAL_ARRIVED;
                        break;
                    }
                    UpdateTraceObject(_timeNow);
                    break;
                
                case TraceCaptureState.EGO_GOAL_ARRIVED:
                    FlushMessages();
                    WriteFile();
                    // if (_traceFormat == TraceFormat.MAUDE)
                        // WriteMaudeFile();
                    // else if (_traceFormat == TraceFormat.YAML)
                        // WriteYamlFile();
                    Debug.Log($"[AWAnalysis] Trace was written to {_filePath}");
                    _state = TraceCaptureState.TRACE_WRITTEN;
                    break;
            }
        }

        protected virtual void UpdateTraceObject(double timeStamp)
        {
            var newState = new StateObject();
            newState.timeStamp = timeStamp;
            
            // ego ground truth
            newState.groundtruth_ego = new EgoGroundTruthObject();
            newState.groundtruth_ego.pose = new Pose2Object();
            newState.groundtruth_ego.pose.position = new Vector3Object(_egoVehicle.Position.x, _egoVehicle.Position.y, _egoVehicle.Position.z);
            var egoRotation = _egoVehicle.Rotation.eulerAngles;
            newState.groundtruth_ego.pose.rotation = new Vector3Object(egoRotation.x, egoRotation.y, egoRotation.z);
            
            newState.groundtruth_ego.twist = new TwistObject();
            newState.groundtruth_ego.twist.linear = new Vector3Object(_egoVehicle.Velocity.x, _egoVehicle.Velocity.y, _egoVehicle.Velocity.z);
            newState.groundtruth_ego.twist.angular = new Vector3Object(_egoVehicle.AngularVelocity.x, _egoVehicle.AngularVelocity.y, _egoVehicle.AngularVelocity.z);
            
            newState.groundtruth_ego.acceleration = new AccelerationObject();
            newState.groundtruth_ego.acceleration.linear = new Vector3Object(_egoVehicle.Acceleration.x, _egoVehicle.Acceleration.y, _egoVehicle.Acceleration.z);
            newState.groundtruth_ego.acceleration.angular = new Vector3Object(_egoVehicle.AngularAcceleration.x, _egoVehicle.AngularAcceleration.y, _egoVehicle.AngularAcceleration.z);
            
            // NPCs ground truth
            int npcCount = CustomNPCSpawningManager.GetNPCs().Count;
            newState.groundtruth_NPCs = new NPCGroundTruthObject[npcCount];
            for (int i = 0; i < npcCount; i++)
            {
                var npc = CustomNPCSpawningManager.GetNPCs()[i];
                newState.groundtruth_NPCs[i] = new NPCGroundTruthObject();
                newState.groundtruth_NPCs[i].name = npc.ScriptName;
                
                newState.groundtruth_NPCs[i].pose = new Pose2Object();
                newState.groundtruth_NPCs[i].pose.position = new Vector3Object(npc.Position.x, npc.Position.y, npc.Position.z);
                newState.groundtruth_NPCs[i].pose.rotation = new Vector3Object(npc.Rotation.x, npc.Rotation.y, npc.Rotation.z);
                
                newState.groundtruth_NPCs[i].twist = new TwistObject();
                newState.groundtruth_NPCs[i].twist.linear = new Vector3Object(npc.Velocity.x, npc.Velocity.y, npc.Velocity.z);
                newState.groundtruth_NPCs[i].twist.angular = new Vector3Object(0, npc.YawAngularSpeed, 0);

                newState.groundtruth_NPCs[i].acceleration = npc.Acceleration;
                
                if (_perceptionMode == PerceptionMode.CAMERA_LIDAR_FUSION)
                {
                    var distanceToEgo = CustomSimUtils.DistanceIgnoreYAxis(npc.Position, _egoVehicle.Position);
                    if (distanceToEgo < _maxDistanceVisibleOnCamera &&
                        CameraUtils.NPCVisibleByCamera(_sensorCamera, npc))
                        newState.groundtruth_NPCs[i].bounding_box = DumpNPCGtBoundingBox(npc);
                }
            }
            
            _traceObject.states.Add(newState);

            int numberOfState = _traceObject.states.Count;

            // 3d detected object by perception module
            while (_objectDetectedMsgs.Count > 0)
            {
                var tuple = _objectDetectedMsgs.Peek();
                if (tuple.Item1 < timeStamp + Time.fixedDeltaTime)
                {
                    int i = numberOfState - 1;
                    for (; i >= Math.Max(0,numberOfState - MAX_LAG_FIXED_STEPS); i--)
                    {
                        StateObject state = _traceObject.states[i];
                        if (tuple.Item1 >= state.timeStamp || i == 0)
                        {
                            state.perception_objects ??= new List<PerceptionObject>();
                            foreach (var detectedObject in tuple.Item2)
                            {
                                var perObject = DumpPerceptionObject2Obj(detectedObject);
                                // check duplicate before adding
                                if (!state.perception_objects.Exists(entry => entry.Equals(perObject)))
                                {
                                    int existObjWithSameId = state.perception_objects.FindIndex(entry => entry.IDEqual(perObject.id));
                                    if (existObjWithSameId == -1)
                                        state.perception_objects.Add(perObject);
                                    else state.perception_objects[existObjWithSameId] = perObject;
                                }
                            }
                            break;
                        }
                    }

                    if (i < Math.Max(0, numberOfState - MAX_LAG_FIXED_STEPS))
                    {
                        Debug.LogError($"A ROS message with timeStamp {tuple.Item1} was discarded " +
                                       $"at {timeStamp} due to its too late.");
                    }
                    _objectDetectedMsgs.Dequeue();
                }
                else break;
            }
            
            // bounding box detected object on camera screen view by perception module
            while (_cameraObjectDetectedMsgs.Count > 0)
            {
                var tuple = _cameraObjectDetectedMsgs.Peek();
                if (tuple.Item1 < timeStamp + Time.fixedDeltaTime)
                {
                    int i = numberOfState - 1;
                    for (; i >= Math.Max(0, numberOfState - MAX_LAG_FIXED_STEPS); i--)
                    {
                        StateObject state = _traceObject.states[i];
                        if (tuple.Item1 >= state.timeStamp || i == 0)
                        {
                            state.boundingbox_perception_objects ??= new List<BBPerceptionObject>();
                            foreach (var detectedObject in tuple.Item2)
                            {
                                var perObject = DumpBBPerceptionObject2Obj(detectedObject);
                                // check duplicate before adding
                                if (!state.boundingbox_perception_objects.Exists(entry => entry.Equals(perObject)))
                                {
                                    state.boundingbox_perception_objects.Add(perObject);
                                }
                            }

                            break;
                        }
                    }
                    if (i < Math.Max(0, numberOfState - MAX_LAG_FIXED_STEPS))
                    {
                        Debug.LogError($"A ROS message with timeStamp {tuple.Item1} was discarded " +
                                       $"at {timeStamp} due to its too late.");
                    }
                    _cameraObjectDetectedMsgs.Dequeue();
                }
                else break;
            }

            WritePlanTrajectoryObj(timeStamp, numberOfState);
            DumpOtherInfo(timeStamp);
        }

        protected PerceptionObject DumpPerceptionObject2Obj(PredictedObject detectedObject)
        {
            PerceptionObject perObj = new PerceptionObject();
            perObj.id = new int[detectedObject.Object_id.Uuid.Length];
            for (int i = 0; i < perObj.id.Length; i++)
            {
                perObj.id[i] = detectedObject.Object_id.Uuid[i];
            }

            perObj.existence_prob = detectedObject.Existence_probability;
            
            perObj.classification = new ClassificationObject[detectedObject.Classification.Length];
            for (int i = 0; i < perObj.classification.Length; i++)
            {
                perObj.classification[i] = new ClassificationObject()
                {
                    label = detectedObject.Classification[i].Label,
                    probability = detectedObject.Classification[i].Probability
                };
            }

            var pos = ROS2Utility.RosMGRSToUnityPosition(detectedObject.Kinematics.Initial_pose_with_covariance.Pose.Position);
            var rot = ROS2Utility.RosToUnityRotation(detectedObject.Kinematics.Initial_pose_with_covariance.Pose.Orientation).eulerAngles;
            perObj.pose = new Pose2Object();
            perObj.pose.position = new Vector3Object(pos.x, pos.y, pos.z);
            perObj.pose.rotation = new Vector3Object(rot.x, rot.y, rot.z);
            
            var vel = detectedObject.Kinematics.Initial_twist_with_covariance.Twist;
            perObj.twist = new TwistObject();
            perObj.twist.linear = new Vector3Object(vel.Linear.X, vel.Linear.Y, vel.Linear.Z);
            perObj.twist.angular = new Vector3Object(vel.Angular.X, vel.Angular.Y, vel.Angular.Z);
                
            var accel = detectedObject.Kinematics.Initial_acceleration_with_covariance.Accel;
            perObj.acceleration = new AccelerationObject();
            perObj.acceleration.linear = new Vector3Object(accel.Linear.X, accel.Linear.Y, accel.Linear.Z);
            perObj.acceleration.angular = new Vector3Object(accel.Angular.X, accel.Angular.Y, accel.Angular.Z);

            switch (detectedObject.Shape.Type)
            {
                case Shape.BOUNDING_BOX:
                    perObj.shape = new BoxDetectedShapeObject()
                    {
                        size = new Vector3Object(detectedObject.Shape.Dimensions.X,
                            detectedObject.Shape.Dimensions.Y,
                            detectedObject.Shape.Dimensions.Z),
                    };
                    break;
                case Shape.POLYGON:
                    Debug.LogWarning("Unhandle the case Detected object's shape type is polygon.");
                    break;
                case Shape.CYLINDER:
                    Debug.LogWarning("Unhandle the case Detected object's shape type is cylinder.");
                    break;
            }
            return perObj;
        }
        
        protected BBPerceptionObject DumpBBPerceptionObject2Obj(DetectedObjectWithFeature detectedObject)
        {
            BBPerceptionObject perObj = new BBPerceptionObject();

            perObj.existence_prob = detectedObject.Object.Existence_probability;
            
            perObj.classification = new ClassificationObject[detectedObject.Object.Classification.Length];
            for (int i = 0; i < perObj.classification.Length; i++)
            {
                perObj.classification[i] = new ClassificationObject()
                {
                    label = detectedObject.Object.Classification[i].Label,
                    probability = detectedObject.Object.Classification[i].Probability
                };
            }
            
            perObj.bounding_box = new BoundingBoxObject()
            {
                x = (int)detectedObject.Feature.Roi.X_offset,
                y = (int)detectedObject.Feature.Roi.Y_offset,
                width = (int)detectedObject.Feature.Roi.Width,
                height = (int)detectedObject.Feature.Roi.Height
            };
            return perObj;
        }

        protected void FlushMessages()
        {
            while (_objectDetectedMsgs.Count > 0)
            {
                var tuple = _objectDetectedMsgs.Dequeue();
                int i = _traceObject.states.Count - 1;
                for (; i >= 0; i--)
                {
                    StateObject state = _traceObject.states[i];
                    if (tuple.Item1 >= state.timeStamp || i == 0)
                    {
                        state.perception_objects ??= new List<PerceptionObject>();
                        foreach (var detectedObject in tuple.Item2)
                        {
                            var perObject = DumpPerceptionObject2Obj(detectedObject);
                            // check duplicate before adding
                            if (!state.perception_objects.Exists(entry => entry.Equals(perObject)))
                            {
                                int existObjWithSameId =
                                    state.perception_objects.FindIndex(entry => entry.IDEqual(perObject.id));
                                if (existObjWithSameId == -1)
                                    state.perception_objects.Add(perObject);
                                else state.perception_objects[existObjWithSameId] = perObject;
                            }
                        }

                        break;
                    }
                }
            }

            while (_cameraObjectDetectedMsgs.Count > 0)
            {
                var tuple = _cameraObjectDetectedMsgs.Dequeue();
                int i = _traceObject.states.Count - 1;
                for (; i >= 0; i--)
                {
                    StateObject state = _traceObject.states[i];
                    if (tuple.Item1 >= state.timeStamp || i == 0)
                    {
                        state.boundingbox_perception_objects ??= new List<BBPerceptionObject>();
                        foreach (var detectedObject in tuple.Item2)
                        {
                            var perObject = DumpBBPerceptionObject2Obj(detectedObject);
                            // check duplicate before adding
                            if (!state.boundingbox_perception_objects.Exists(entry => entry.Equals(perObject)))
                            {
                                state.boundingbox_perception_objects.Add(perObject);
                            }
                        }
                        break;
                    }
                }
            }
        }

        protected abstract void WriteFile();

        protected void DumpVehicleDetails()
        {
            // ego details
            MeshFilter meshFilter = _autowareEgoCar.GetComponentInChildren<MeshFilter>();
            
            _traceObject.ego_detail = new EgoDetailObject()
            {
                center = new Vector3Object(meshFilter.mesh.bounds.center + meshFilter.transform.parent.parent.localPosition),
                extents = new Vector3Object(meshFilter.mesh.bounds.extents)
            };
            
            // NPCs details
            var npcs = CustomNPCSpawningManager.GetNPCs();
            _traceObject.npcs_detail = new NPCDetailObject[npcs.Count];
            for (int i = 0; i < npcs.Count; i++)
            {
                _traceObject.npcs_detail[i] = new NPCDetailObject()
                {
                    name = npcs[i].ScriptName,
                    center = new Vector3Object(npcs[i].Bounds.center),
                    extents = new Vector3Object(npcs[i].Bounds.extents)
                };
            }
        }
        
        // start capturing traces, and also register various ROS events
        protected void SubscribeRosEvents()
        {
            // Debug.Log("[AWAnalysis] Start capturing trace");
            SimulatorROS2Node.CreateSubscription<PredictedObjects>(
                TopicName.TOPIC_PERCEPTION_RECOGNITION_OBJECTS, msg =>
                {
                    if (msg.Objects.Length > 0)
                        _objectDetectedMsgs.Enqueue(new Tuple<double, PredictedObject[]>(
                            msg.Header.Stamp.Sec + msg.Header.Stamp.Nanosec / Math.Pow(10, 9),
                            msg.Objects));
                });

            if (_perceptionMode == PerceptionMode.CAMERA_LIDAR_FUSION)
            {
                SimulatorROS2Node.CreateSubscription<DetectedObjectsWithFeature>(
                    TopicName.TOPIC_PERCEPTION_CAMERA_OBJECTS, msg =>
                    {
                        if (msg.Feature_objects.Length > 0)
                            _cameraObjectDetectedMsgs.Enqueue(new Tuple<double, DetectedObjectWithFeature[]>(
                                msg.Header.Stamp.Sec + msg.Header.Stamp.Nanosec / Math.Pow(10, 9),
                                msg.Feature_objects));
                    });
            }

            // log when the Ego vehicle arrives its goal
            routeStateSubscriber = SimulatorROS2Node.CreateSubscription<RouteState>(
                TopicName.TOPIC_API_ROUTING_STATE, msg =>
                {
                    if (msg.State == RouteState.ARRIVED)
                        _state = TraceCaptureState.EGO_GOAL_ARRIVED;
                });
            
            // capture planning trajectory
            if (ConfigLoader.CapturePlanTrajectory())
            {
                SimulatorROS2Node.CreateSubscription<autoware_planning_msgs.msg.Trajectory>(
                    TopicName.TOPIC_PLANNING_TRAJECTORY, msg =>
                    {
                        _planTrajectoryMsgs.Enqueue(new Tuple<double, awTrajectoryPoint[]>(
                            msg.Header.Stamp.Sec + msg.Header.Stamp.Nanosec / Math.Pow(10, 9),
                            msg.Points));
                    });
            }
        }

        /// <summary>
        /// return the bounding box of `npc`.
        /// </summary>
        /// <param name="npc"></param>
        /// <returns></returns>
        protected BoundingBoxObject DumpNPCGtBoundingBox(NPCVehicle npc)
        {
            MeshCollider bodyCollider = npc.GetComponentInChildren<MeshCollider>();
            Vector3 localPosition = bodyCollider.transform.parent.localPosition;

            Mesh mesh = bodyCollider.sharedMesh;
            Vector3[] localVertices = mesh.vertices;

            var worldVertices = new List<Vector3>();
            for (int i = 0; i < localVertices.Length; i++)
                worldVertices.Add(npc.transform.TransformPoint(
                    localVertices[i] + localPosition));

            var screenVertices = new List<Vector3>();
            for (int i = 0; i < worldVertices.Count; i++)
            {
                var screenPoint = _sensorCamera.WorldToScreenPoint(worldVertices[i]);
                if (screenPoint.z > 0)
                {
                    screenPoint = CameraUtils.FixScreenPoint(screenPoint, _sensorCamera);
                    screenVertices.Add(screenPoint);
                }
            }
            float min_x = screenVertices[0].x;
            float min_y = screenVertices[0].y;
            float max_x = screenVertices[0].x;
            float max_y = screenVertices[0].y;

            for (int i = 1; i < screenVertices.Count; i++)
            {
                if (screenVertices[i].x < min_x &&
                    CameraUtils.InRange(screenVertices[i].y, 0, _sensorCamera.pixelHeight))
                    min_x = screenVertices[i].x;
                if (screenVertices[i].y < min_y &&
                    CameraUtils.InRange(screenVertices[i].x, 0, _sensorCamera.pixelWidth))
                    min_y = screenVertices[i].y;
                if (screenVertices[i].x > max_x &&
                    CameraUtils.InRange(screenVertices[i].y, 0, _sensorCamera.pixelHeight))
                    max_x = screenVertices[i].x;
                if (screenVertices[i].y > max_y &&
                    CameraUtils.InRange(screenVertices[i].x, 0, _sensorCamera.pixelWidth))
                    max_y = screenVertices[i].y;
            }
            // if min_x is -0.1
            min_x = Mathf.Max(0, min_x);
            min_y = Mathf.Max(0, min_y);
            max_x = Mathf.Min(_sensorCamera.pixelWidth, max_x);
            max_y = Mathf.Min(_sensorCamera.pixelHeight, max_y);
            return new BoundingBoxObject()
            {
                x = min_x,
                y = min_y,
                width = max_x - min_x,
                height = max_y - min_y
            };
        }

        protected void WritePlanTrajectoryObj(double timeStamp, int numberOfState)
        {
            while (_planTrajectoryMsgs.Count > 0)
            {
                var tuple = _planTrajectoryMsgs.Dequeue();
                if (tuple.Item1 < timeStamp + Time.fixedDeltaTime)
                {
                    int i = numberOfState - 1;
                    for (; i >= Math.Max(0, numberOfState - MAX_LAG_FIXED_STEPS); i--)
                    {
                        StateObject state = _traceObject.states[i];
                        if (tuple.Item1 >= state.timeStamp || i == 0)
                        {
                            state.plan_trajectory = new PlanTrajectory()
                            {
                                points = DumpTrajectoryPoints(tuple.Item2)
                            };
                            break;
                        }
                    }
                    if (i < Math.Max(0, numberOfState - MAX_LAG_FIXED_STEPS))
                    {
                        Debug.LogError($"A ROS message with timeStamp {tuple.Item1} was discarded " +
                                       $"at {timeStamp} due to its too late.");
                    }
                }
                else break;
            }
        }

        // write info related to cut-in, cut-out, deceleration
        protected void DumpOtherInfo(double timeStamp)
        {
            if (_traceObject.other is CutInInfoObject cutInInfo &&
                cutInInfo.time_cutin_start == 0)
            {
                var innerState = CustomNPCSpawningManager.CutInNPCInternalState();
                if (innerState != null &&
                    innerState.CurrentFollowingLane.name == innerState.CustomConfig.LaneChange.TargetLane &&
                    innerState.WaypointIndex == innerState.CustomConfig.LaneChange.TargetLaneWaypointIndex)
                {
                    cutInInfo.time_cutin_start = timeStamp;
                    cutInInfo.cutin_npc_name = CustomNPCSpawningManager.GetCutInVehicle().ScriptName;
                }
            }
            else if (_traceObject.other is CutOutInfoObject cutOutInfo &&
                     cutOutInfo.time_cutout_start == 0)
            {
                var innerState = CustomNPCSpawningManager.CutOutNPCInternalState();
                if (innerState != null &&
                    innerState.CurrentFollowingLane.name == innerState.CustomConfig.LaneChange.TargetLane &&
                    innerState.WaypointIndex == innerState.CustomConfig.LaneChange.TargetLaneWaypointIndex)
                {
                    cutOutInfo.time_cutout_start = timeStamp;
                    cutOutInfo.cutout_npc_name = CustomNPCSpawningManager.GetCutOutVehicle().ScriptName;
                }
            }
            else if (_traceObject.other is DecelerationInfoObject decelInfo &&
                     decelInfo.time_deceleration_start == 0)
            {
                var innerState = CustomNPCSpawningManager.DecelerationNPCInternalState();
                if (innerState != null &&
                    innerState.SpeedMode == NPCVehicleSpeedMode.STOP)
                {
                    decelInfo.time_deceleration_start = timeStamp;
                    decelInfo.deceleration_npc_name = CustomNPCSpawningManager.GetDecelerationVehicle().ScriptName;
                }
            }
        }

        protected TrajectoryPoint[] DumpTrajectoryPoints(awTrajectoryPoint[] points)
        {
            int no = Math.Min(points.Length, ConfigLoader.Config().PlanTrajectoryMaxStepsRecording);
            var result = new TrajectoryPoint[no];
            for (int i = 0; i < no; i++)
            {
                result[i] = new TrajectoryPoint()
                {
                    longitudinal_velocity = points[i].Longitudinal_velocity_mps,
                    lateral_velocity = points[i].Lateral_velocity_mps,
                    acceleration = points[i].Acceleration_mps2,
                    time_from_start = points[i].Time_from_start.Sec +
                                      points[i].Time_from_start.Nanosec / Math.Pow(10, 9),
                    pose = new PoseObject()
                };
                var unityPoint = ROS2Utility.RosMGRSToUnityPosition(points[i].Pose.Position);
                var unityRotation = ROS2Utility.RosToUnityRotation(points[i].Pose.Orientation);
                result[i].pose.position = new Vector3Object(unityPoint.x, unityPoint.y, unityPoint.z);
                result[i].pose.quaternion = new QuaternionObject(unityRotation.x, unityRotation.y, unityRotation.z, unityRotation.w);
            }
            return result;
        }

        protected bool ValidateFilePath()
        {
            try
            {
                var dirPath = Path.GetDirectoryName(_filePath);
                var filename = Path.GetFileName(_filePath);
                if (!filename.Contains("."))
                {
                    if (this is MaudeTraceWriter)
                        filename += ".maude";
                    else if (this is YamlTraceWriter)
                        filename += ".yaml";
                    _filePath = Path.Combine(dirPath, filename);
                }

                return true;
            }
            catch (ArgumentException e)
            {
                Debug.LogError($"The path for saving trace file `{_filePath}` seems to be invalid. Exception: {e.Message}");
            }
            return false;
        }
    }

    public enum TraceCaptureState
    {
        INITIALIZED,
        // LOCALIZATION_INITIALIZED,
        AUTO_MODE_READY,
        READY_TO_CAPTURE,
        TRACE_CAPTURING,
        EGO_GOAL_ARRIVED,
        TRACE_WRITTEN
    }
}