using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using System.IO;
using System.Linq;
using AWSIM.AWAnalysis.CustomSim;
using autoware_adapi_v1_msgs.msg;
using autoware_perception_msgs.msg;
using autoware_vehicle_msgs.msg;
using tier4_perception_msgs.msg;
using ROS2;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class TraceWriter
    {
        public const string TEMPLATE = "in ../base.maude\n\nmod TRACE is " +
            "\n  pr FORMULAS .\n\n";

        private readonly string _filePath;
        private readonly Vehicle _egoVehicle;
        private readonly Camera _sensorCamera;
        private readonly TraceCaptureConfig _config;
        private readonly PerceptionMode _perceptionMode;
        private readonly float _maxDistanceVisibleOnCamera;

        // inner use
        private TraceCaptureState _state;
        private string contents;
        private float _timeNow;
        // ROS time at start
        // When AWSIM starts after Autoware, it is 0.
        // When AWSIM starts before Autoware, rosTimeAtStart > 0.
        private double _rosTimeAtStart;
        // time when autonomous operation mode becomes ready
        private float _autoOpModeReadyTime = -1f;
        
        private Queue<Tuple<double, PredictedObject[]>> _objectDetectedMsgs;
        private Queue<Tuple<double, DetectedObjectWithFeature[]>> _cameraObjectDetectedMsgs;
        private Tuple<double,IEnumerable<string>> lastGroundTruthState;
        private Tuple<double,IEnumerable<string>> preLastGroundTruthState;
        
        ISubscription<OperationModeState> opModeSubscriber;
        ISubscription<RouteState> routeStateSubscriber;
        ISubscription<LocalizationInitializationState> localizationInitStateSubscriber;
        
        public TraceWriter(string filePath, Vehicle egoVehicle, Camera sensorCamera, PerceptionMode perceptionMode)
        {
            this._filePath = filePath;
            this._egoVehicle = egoVehicle;
            _sensorCamera = sensorCamera;
            this._config = new TraceCaptureConfig(CaptureStartingTime.AW_AUTO_MODE_READY);
            _perceptionMode = perceptionMode;
            contents = TEMPLATE;
            _objectDetectedMsgs = new Queue<Tuple<double, PredictedObject[]>>();
            _cameraObjectDetectedMsgs = new Queue<Tuple<double, DetectedObjectWithFeature[]>>();
            _maxDistanceVisibleOnCamera = ConfigLoader.Config().MaxDistanceVisibleonCamera;
        }

        public TraceWriter(string filePath, Vehicle egoVehicle, Camera sensorCamera,
            TraceCaptureConfig config, PerceptionMode perceptionMode)
            : this(filePath, egoVehicle, sensorCamera, perceptionMode)
        {
            _config = config;
        }

        public void Start()
        {
            _state = TraceCaptureState.INITIALIZED;
            // difference between ROS time and Unity time
            var rosTime = SimulatorROS2Node.GetCurrentRosTime();
            _rosTimeAtStart = rosTime.Sec + rosTime.Nanosec / Math.Pow(10, 9);
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
                    contents += $"  eq startTime = {_timeNow + _rosTimeAtStart} .\n" +
                                $"  eq fixedTimestep = {(int)(Time.fixedDeltaTime * 1000)} .\n" +
                                $"  eq init = ";
                    _state = TraceCaptureState.TRACE_CAPTURING;
                    Debug.Log("[AWAnalysis] Start capturing trace");
                    break;
                
                case TraceCaptureState.TRACE_CAPTURING:
                    double timeStamp = _timeNow + _rosTimeAtStart;
                    string stateStr = $"{timeStamp} # {{";
                    // Dump ground truth trace of Ego and NPCs
                    stateStr += DumpEgoInfo();
                    List<NPCVehicle> npcs = CustomNPCSpawningManager.GetNPCs();
                    npcs.ForEach(npc => stateStr += ", " + DumpNPCInfo(npc));

                    while (_objectDetectedMsgs.Count > 0)
                    {
                        var tuple = _objectDetectedMsgs.Peek();
                        if (lastGroundTruthState == null ||
                             preLastGroundTruthState == null ||
                             tuple.Item1 > lastGroundTruthState.Item1)
                            break;
                        
                        List<string> objectsStr = new List<string>();
                        foreach (var detectedObj in tuple.Item2)
                        {
                            string objStr = DumpDetectedObject(detectedObj);
                            if (!objectsStr.Contains(objStr))
                                objectsStr.Add(objStr);
                        }
                        if (Math.Abs(tuple.Item1 - preLastGroundTruthState.Item1) <=
                            lastGroundTruthState.Item1 - tuple.Item1)
                        {
                            preLastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                                preLastGroundTruthState.Item1,
                                preLastGroundTruthState.Item2.Union(objectsStr));
                        }
                        else
                        {
                            lastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                                lastGroundTruthState.Item1,
                                lastGroundTruthState.Item2.Union(objectsStr));
                        }
                        _objectDetectedMsgs.Dequeue();
                    }
                    
                    while (_cameraObjectDetectedMsgs.Count > 0)
                    {
                        var tuple = _cameraObjectDetectedMsgs.Peek();
                        if (lastGroundTruthState == null ||
                            preLastGroundTruthState == null ||
                            tuple.Item1 > lastGroundTruthState.Item1)
                            break;
                        List<string> objectsStr = new List<string>();
                        foreach (var detectedObject in tuple.Item2)
                        {
                            string objStr = DumpCameraDetectedObject(detectedObject);
                            if (!objectsStr.Contains(objStr))
                                objectsStr.Add(objStr);
                        }
                        if (Math.Abs(tuple.Item1 - preLastGroundTruthState.Item1) <=
                            lastGroundTruthState.Item1 - tuple.Item1)
                        {
                            preLastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                                preLastGroundTruthState.Item1,
                                preLastGroundTruthState.Item2.Union(objectsStr));
                        }
                        else
                        {
                            lastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                                lastGroundTruthState.Item1,
                                lastGroundTruthState.Item2.Union(objectsStr));
                        }
                        _cameraObjectDetectedMsgs.Dequeue();
                    }

                    if (preLastGroundTruthState == null)
                        preLastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                            timeStamp, 
                            new List<string>{stateStr});
                    else if (lastGroundTruthState == null)
                        lastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                            timeStamp, 
                            new List<string>{stateStr});
                    else
                    {
                        string preLastStateStr = preLastGroundTruthState.Item2.First();
                        foreach (string objStr in preLastGroundTruthState.Item2.Skip(1))
                            preLastStateStr += ", " + objStr;
                        // don't forget to add a closing bracket
                        contents += $"{preLastStateStr}}} .\n  rl {preLastStateStr}}}\n  => ";
                        preLastGroundTruthState = lastGroundTruthState;
                        lastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                            timeStamp, 
                            new List<string>{stateStr});
                    }
                    break;
                
                case TraceCaptureState.EGO_GOAL_ARRIVED:
                    WriteFile();
                    Debug.Log($"[AWAnalysis] Trace was written to {_filePath}");
                    _state = TraceCaptureState.TRACE_WRITTEN;
                    break;
            }
        }

        private void WriteFile()
        {
            while (_objectDetectedMsgs.Count > 0)
            {
                var tuple = _objectDetectedMsgs.Dequeue();
                
                List<string> listObjectStr = new List<string>();
                foreach (var detectedObject in tuple.Item2)
                {
                    string str = DumpDetectedObject(detectedObject);
                    if (!listObjectStr.Contains(str))
                        listObjectStr.Add(str);
                }
                if (Math.Abs(tuple.Item1 - preLastGroundTruthState.Item1) <=
                    lastGroundTruthState.Item1 - tuple.Item1)
                {
                    preLastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                        preLastGroundTruthState.Item1,
                        preLastGroundTruthState.Item2.Union(listObjectStr));
                }
                else
                {
                    lastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                        lastGroundTruthState.Item1,
                        lastGroundTruthState.Item2.Union(listObjectStr));
                }
            }
            
            while (_cameraObjectDetectedMsgs.Count > 0)
            {
                var tuple = _cameraObjectDetectedMsgs.Dequeue();
                
                List<string> listObjectStr2 = new List<string>();
                // string objectsStr = "";
                foreach (var detectedObject in tuple.Item2)
                {
                    string str = DumpCameraDetectedObject(detectedObject);
                    if (!listObjectStr2.Contains(str))
                        listObjectStr2.Add(str);
                }
                if (Math.Abs(tuple.Item1 - preLastGroundTruthState.Item1) <=
                    lastGroundTruthState.Item1 - tuple.Item1)
                {
                    preLastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                        preLastGroundTruthState.Item1,
                        preLastGroundTruthState.Item2.Union(listObjectStr2));
                }
                else
                {
                    lastGroundTruthState = new Tuple<double, IEnumerable<string>>(
                        lastGroundTruthState.Item1,
                        lastGroundTruthState.Item2.Union(listObjectStr2));
                }
            }
            
            string preLastStateStr = preLastGroundTruthState.Item2.First();
            foreach (string objStr in preLastGroundTruthState.Item2.Skip(1))
                preLastStateStr += ", " + objStr;
            
            string lastStateStr = lastGroundTruthState.Item2.First();
            foreach (string objStr in lastGroundTruthState.Item2.Skip(1))
                lastStateStr += ", " + objStr;
                
            contents += $"{preLastStateStr}}} .\n  rl {preLastStateStr}}}\n  => ";
            contents += $"{lastStateStr}}} .\n";
            // loop rule
            contents += $"  rl {lastStateStr}}} \n  => {lastStateStr}}} .\n";
            if (_perceptionMode == PerceptionMode.CAMERA_LIDAR_FUSION)
                // write camera info
                contents += $"  eq cameraScreenWidth = {_sensorCamera.pixelWidth} .\n" +
                            $"  eq cameraScreenHeight = {_sensorCamera.pixelHeight} .\n";
            contents += "endm";
            File.WriteAllText(_filePath, contents);
        }

        // start capturing traces, and also register various ROS events
        private void SubscribeRosEvents()
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
        }
        
        private string DumpEgoInfo()
        {
            string pose = $"pose: {{pos: {_egoVehicle.Position.x} {_egoVehicle.Position.y} {_egoVehicle.Position.z}, qua: {_egoVehicle.Rotation.x} {_egoVehicle.Rotation.y} {_egoVehicle.Rotation.z} | {_egoVehicle.Rotation.w}}}";
            string twist = $"twist: {{lin: {_egoVehicle.Velocity.x} {_egoVehicle.Velocity.y} {_egoVehicle.Velocity.z}, ang: {_egoVehicle.AngularVelocity.x} {_egoVehicle.AngularVelocity.y} {_egoVehicle.AngularVelocity.z}}}";
            string accel = $"accel: {{lin: {_egoVehicle.Acceleration.x} {_egoVehicle.Acceleration.y} {_egoVehicle.Acceleration.z}, ang: {_egoVehicle.AngularAcceleration.x} {_egoVehicle.AngularAcceleration.y} {_egoVehicle.AngularAcceleration.z}}}";
            return $"{{name: \"ego\", {pose}, {twist}, {accel}}}";
        }

        private string DumpNPCInfo(NPCVehicle npc)
        {
            var centerPos = npc.CenterPosition();
            string pose = $"pose: {{pos: {centerPos.x} {centerPos.y} {centerPos.z}, rota: {npc.EulerAnguleY}}}";
            string twist = $"twist: {{lin: {npc.Velocity.x} {npc.Velocity.y} {npc.Velocity.z}, ang: {npc.YawAngularSpeed}}}";
            string accel = $"accel: {npc.Acceleration}";
            if (_perceptionMode == PerceptionMode.CAMERA_LIDAR_FUSION)
            {
                var distanceToEgo = CustomSimUtils.DistanceIgnoreYAxis(npc.Position, _egoVehicle.Position);
                if (distanceToEgo < _maxDistanceVisibleOnCamera &&
                    CameraUtils.NPCVisibleByCamera(_sensorCamera, npc))
                    return
                        $"{{name: \"{npc.ScriptName ?? ""}\", {pose}, {twist}, {accel}, {DumpNPCGtBoundingBox(npc)}}}";
            }
            return $"{{name: \"{npc.ScriptName ?? ""}\", {pose}, {twist}, {accel}}}";
        }
        
        /// <summary>
        /// return the bounding box of `npc`.
        /// </summary>
        /// <param name="npc"></param>
        /// <returns></returns>
        private string DumpNPCGtBoundingBox(NPCVehicle npc)
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
            return $"gt-rect: {min_x} {min_y} {max_x} {max_y}";
        }

        private string DumpDetectedObject(PredictedObject obj)
        {
            string uuid = "";
            for (int i = 0; i < obj.Object_id.Uuid.Length; i++)
            {
                uuid += $"{(int)obj.Object_id.Uuid[i]} ";
            }
            if (uuid != "")
                uuid = uuid[..^1];

            string classification = "";
            foreach (var t in obj.Classification)
            {
                classification += $"{(int)t.Label} -> {t.Probability}, ";
            }
            if (classification != "")
                classification = classification[..^2];

            var pos = ROS2Utility.RosMGRSToUnityPosition(obj.Kinematics.Initial_pose_with_covariance.Pose.Position);
            var rot = ROS2Utility.RosToUnityRotation(obj.Kinematics.Initial_pose_with_covariance.Pose.Orientation).eulerAngles;
            string pose = $"pose: {{pos: {pos.x} {pos.y} {pos.z}, rota: {rot.x} {rot.y} {rot.z}}}";

            var vel = obj.Kinematics.Initial_twist_with_covariance.Twist;
            string twist = $"twist: {{lin: {vel.Linear.X} {vel.Linear.Y} {vel.Linear.Z}, ang: {vel.Angular.X} {vel.Angular.Y} {vel.Angular.Z}}}";

            var accel = obj.Kinematics.Initial_acceleration_with_covariance.Accel;
            string accelStr = $"accel: {{lin: {accel.Linear.X} {accel.Linear.Y} {accel.Linear.Z}, ang: {accel.Angular.X} {accel.Angular.Y} {accel.Angular.Z}}}";

            return $"{{id: [{uuid}], epro: {obj.Existence_probability}, class: [{classification}], {pose}, {twist}, {accelStr}}}";
        }
        
        private string DumpCameraDetectedObject(DetectedObjectWithFeature obj)
        {
            string classification = "";
            foreach (var t in obj.Object.Classification)
            {
                classification += $"{(int)t.Label} -> {t.Probability}, ";
            }
            if (classification != "")
                classification = classification[..^2];

            var roi = obj.Feature.Roi;
            string rectStr = $"rect: {roi.X_offset} {_sensorCamera.pixelHeight - roi.Y_offset - roi.Height} {roi.Width} {roi.Height}";

            return $"{{epro: {obj.Object.Existence_probability}, class: [{classification}], {rectStr}}}";
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