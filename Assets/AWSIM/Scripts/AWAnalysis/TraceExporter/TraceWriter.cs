using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using AWSIM.AWAnalysis.CustomSim;
using autoware_adapi_v1_msgs.msg;
using autoware_perception_msgs.msg;
using autoware_vehicle_msgs.msg;
using AWSIM_Script.Object;
using AWSIM.AWAnalysis.Monitor;
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
            _egoVehicle = EgoSingletonInstance.AutowareEgoVehicle;
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

            if (CustomSimManager.GetCutInVehicle() != null)
                _traceObject.other = new CutInInfoObject();
            else if (CustomSimManager.GetCutOutVehicle() != null)
                _traceObject.other = new CutOutInfoObject();
            else if (CustomSimManager.GetDecelerationVehicle() != null)
                _traceObject.other = new DecelerationInfoObject();
            else if (CustomSimManager.GetSwerveVehicle() != null)
                _traceObject.other = new SwerveInfoObject();
            else if (CustomSimManager.GetUTurnVehicle() != null)
                _traceObject.other = new UTurnInfoObject();
        }
        
        public void Start()
        {
            _state = TraceCaptureState.NOT_YET_CAPTURE;
            // // difference between ROS time and Unity time
            // // updated: since we use Unity time source, this is no longer needed
            // // var rosTime = SimulatorROS2Node.GetCurrentRosTime();
            // // _rosTimeAtStart = rosTime.Sec + rosTime.Nanosec / Math.Pow(10, 9);
        }
        
        private bool IsReadyToCapture()
        {
            switch (_config.TraceCaptureFrom)
            {
                case CaptureStartingTime.AW_AUTO_MODE_READY:
                    return ExecutionStateTracker.State >= ExecutionState.AUTO_MODE_READY &&
                           _timeNow >= ExecutionStateTracker.AutoOpModeReadyTime +
                                        ConfigLoader.Config().DelaySendingEngageCmd;
                case CaptureStartingTime.AW_LOCALIZATION_INITIALIZED:
                    return ExecutionStateTracker.State >= ExecutionState.LOCALIZATION_SUCCEEDED;
                default:
                    return true;
            }
        }

        public void Update()
        {
            _timeNow = Time.fixedTime;
            switch (_state)
            {
                case TraceCaptureState.NOT_YET_CAPTURE:
                    if (IsReadyToCapture())
                    {
                        SubscribeRosEvents();
                        _state = TraceCaptureState.CAPTURING;
                    }
                    break;
                case TraceCaptureState.CAPTURING:
                    // if saving-timeout is reached
                    if (!Mathf.Approximately(_config.SavingTimeout, Simulation.DUMMY_SAVING_TIMEOUT) &&
                        _timeNow > _config.SavingTimeout + _startTime)
                    {
                        _traceObject.comment = "Timeout reached before Ego arrives goal.";
                        FlushMessages();
                        WriteFile();
                        Debug.Log($"[AWAnalysis] Trace was written to {_filePath}");

                        _state = TraceCaptureState.DONE;
                        break;
                    }
                    UpdateTraceObject(_timeNow);
                    break;
            }
        }

        protected virtual void UpdateTraceObject(double timeStamp)
        {
            var newState = new StateObject();
            newState.timeStamp = timeStamp;
            
            // ego ground truth
            newState.groundtruth_ego = StatusExtraction.ExtractEgoKinematic(_egoVehicle);
            
            // NPC vehicles ground truth
            var npcVehicles = CustomSimManager.GetNPCs();
            newState.groundtruth_NPCs = StatusExtraction.ExtractNPCKinematics(npcVehicles);
            if (_perceptionMode == PerceptionMode.CAMERA_LIDAR_FUSION)
            {
                var bboxes = StatusExtraction.Extract2DVehicleBoundingBoxes(
                    npcVehicles, _sensorCamera, _egoVehicle, _maxDistanceVisibleOnCamera);
                for (int i = 0; i < newState.groundtruth_NPCs.Length; i++)
                {
                    if (bboxes[i] != null)
                        newState.groundtruth_NPCs[i].bounding_box = bboxes[i];
                }
            }

            // pedestrians
            var npcPedestrians = CustomSimManager.GetPedestrians();
            newState.groundtruth_pedestrians = StatusExtraction.ExtractPedestrians(npcPedestrians);
            if (_perceptionMode == PerceptionMode.CAMERA_LIDAR_FUSION)
            {
                var bboxes = StatusExtraction.Extract2DPedestrianBoundingBoxes(
                    npcPedestrians, _sensorCamera, _egoVehicle, _maxDistanceVisibleOnCamera);
                for (int i = 0; i < newState.groundtruth_pedestrians.Length; i++)
                {
                    if (bboxes[i] != null)
                        newState.groundtruth_pedestrians[i].bounding_box = bboxes[i];
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

            perObj.pose = DumpPose(detectedObject.Kinematics.Initial_pose_with_covariance.Pose);

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
                        size = new Vector3Object(detectedObject.Shape.Dimensions.Y,
                            detectedObject.Shape.Dimensions.Z,
                            detectedObject.Shape.Dimensions.X),
                    };
                    break;
                case Shape.POLYGON:
                    var polygonShape = new PolygonDetectedShapeObject()
                    {
                        footprint = new Vector3Object[detectedObject.Shape.Footprint.Points.Length]
                    };
                    for (int i = 0; i < detectedObject.Shape.Footprint.Points.Length; i++)
                    {
                        polygonShape.footprint[i] = new Vector3Object(
                            detectedObject.Shape.Footprint.Points[i].X,
                            detectedObject.Shape.Footprint.Points[i].Y,
                            detectedObject.Shape.Footprint.Points[i].Z);
                    };
                    perObj.shape = polygonShape;
                    break;
                case Shape.CYLINDER:
                    Debug.LogWarning("Unhandle the case Detected object's shape type is cylinder.");
                    break;
            }
            
            // prediction traveling paths
            if (ConfigLoader.CapturePredictionPaths())
            {
                perObj.predict_paths = new PredictPathObject[detectedObject.Kinematics.Predicted_paths.Length];
                for (int i = 0; i < perObj.predict_paths.Length; i++)
                {
                    perObj.predict_paths[i] = new PredictPathObject()
                    {
                        confidence = detectedObject.Kinematics.Predicted_paths[i].Confidence,
                        time_step = TimestampToDouble(detectedObject.Kinematics.Predicted_paths[i].Time_step)
                    };
                    perObj.predict_paths[i].path =
                        new Pose2Object[detectedObject.Kinematics.Predicted_paths[i].Path.Length];
                    for (int j = 0; j < perObj.predict_paths[i].path.Length; j++)
                    {
                        perObj.predict_paths[i].path[j] =
                            DumpPose(detectedObject.Kinematics.Predicted_paths[i].Path[j]);
                    }
                }
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
            var npcs = CustomSimManager.GetNPCs();
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
                var innerState = CustomSimManager.CutInNPCInternalState();
                if (innerState != null &&
                    innerState.CurrentFollowingLane.name == innerState.CustomConfig.LaneChange.TargetLane &&
                    innerState.WaypointIndex == innerState.CustomConfig.LaneChange.TargetLaneWaypointIndex)
                {
                    cutInInfo.time_cutin_start = timeStamp;
                    cutInInfo.cutin_npc_name = CustomSimManager.GetCutInVehicle().ScriptName;
                }
            }
            else if (_traceObject.other is CutOutInfoObject cutOutInfo &&
                     cutOutInfo.time_cutout_start == 0)
            {
                var innerState = CustomSimManager.CutOutNPCInternalState();
                if (innerState != null &&
                    innerState.CurrentFollowingLane.name == innerState.CustomConfig.LaneChange.TargetLane &&
                    innerState.WaypointIndex == innerState.CustomConfig.LaneChange.TargetLaneWaypointIndex)
                {
                    cutOutInfo.time_cutout_start = timeStamp;
                    cutOutInfo.cutout_npc_name = CustomSimManager.GetCutOutVehicle().ScriptName;
                }
            }
            else if (_traceObject.other is DecelerationInfoObject decelInfo &&
                     decelInfo.time_deceleration_start == 0)
            {
                var innerState = CustomSimManager.DecelerationNPCInternalState();
                if (innerState != null &&
                    innerState.SpeedMode == NPCVehicleSpeedMode.STOP)
                {
                    decelInfo.time_deceleration_start = timeStamp;
                    decelInfo.deceleration_npc_name = CustomSimManager.GetDecelerationVehicle().ScriptName;
                }
            }
            else if (_traceObject.other is SwerveInfoObject swerveInfo &&
                     swerveInfo.time_swerve_start == 0)
            {
                var innerState = CustomSimManager.SwerveNPCInternalState();
                if (innerState != null &&
                    innerState.CurrentFollowingLane.OriginName() == innerState.CustomConfig.LateralWandering.SourceLane &&
                    innerState.WaypointIndex == innerState.CustomConfig.LateralWandering.SourceLaneWaypointIndex + 1)
                {
                    swerveInfo.time_swerve_start = Math.Round(timeStamp + Time.fixedDeltaTime, 3);
                    swerveInfo.swerve_npc_name = CustomSimManager.GetSwerveVehicle().ScriptName;
                }
            }
            else if (_traceObject.other is UTurnInfoObject uTurnInfo &&
                     uTurnInfo.time_uturn_start == 0)
            {
                var innerState = CustomSimManager.UTurnNPCInternalState();
                if (innerState != null &&
                    innerState.CurrentFollowingLane.OriginName() == innerState.CustomConfig.UTurn.SourceLane &&
                    innerState.WaypointIndex == innerState.CustomConfig.UTurn.SourceLaneWaypointIndex + 1)
                {
                    uTurnInfo.time_uturn_start = Math.Round(timeStamp + Time.fixedDeltaTime, 3);
                    uTurnInfo.uturn_npc_name = CustomSimManager.GetUTurnVehicle().ScriptName;
                }
            }
        }

        protected TrajectoryPoint[] DumpTrajectoryPoints(awTrajectoryPoint[] points)
        {
            var result = new TrajectoryPoint[points.Length];
            for (int i = 0; i < points.Length; i++)
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

        protected Pose2Object DumpPose(geometry_msgs.msg.Pose pose)
        {
            var pos = ROS2Utility.RosMGRSToUnityPosition(pose.Position);
            var rot = ROS2Utility.RosToUnityRotation(pose.Orientation).eulerAngles;
            return new Pose2Object()
            {
                position = new Vector3Object(pos.x, pos.y, pos.z),
                rotation = new Vector3Object(rot.x, rot.y, rot.z)
            };
        }

        protected double TimestampToDouble(builtin_interfaces.msg.Duration timestamp)
        {
            return timestamp.Sec + timestamp.Nanosec / Math.Pow(10, 9);
        }
    }

    public enum TraceCaptureState
    {
        NOT_YET_CAPTURE = 0,
        CAPTURING,
        DONE
    }
}