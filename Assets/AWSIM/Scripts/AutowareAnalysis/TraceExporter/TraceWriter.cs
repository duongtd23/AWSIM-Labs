using System;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using AWSIM.AWAnalysis.CustomSim;
using autoware_adapi_v1_msgs.msg;
using autoware_perception_msgs.msg;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class TraceWriter
    {
        public const string TEMPLATE = "in base.maude\n\nmod TRACE is " +
            "\n  pr AWSTATE .\n\n  eq init = ";

        private string filePath;
        private Vehicle egoVehicle;
        private TraceCaptureConfig config;
        
        // inner use
        private string contents;
        private bool ready, fileWritten;
        private float timeStart;
        private float timeNow;
        private Queue<Tuple<double, PredictedObject[]>> objectDetectedMsgs;
        private Tuple<double,string> lastGroundTruthState;
        private Tuple<double,string> preLastGroundTruthState;
        private float autoOpModeReadTime = -1f;

        // ROS time at start
        private double rosTimeAtStart;

        // public const int CAPTURE_DURATION = 60;

        public TraceWriter(string filePath, Vehicle egoVehicle)
        {
            this.filePath = filePath;
            this.egoVehicle = egoVehicle;
            this.config = new TraceCaptureConfig(CaptureStartingTime.AW_LOCALIZATION_INITIALIZED);
            contents = TEMPLATE;
            objectDetectedMsgs = new Queue<Tuple<double, PredictedObject[]>>();
        }

        public TraceWriter(string filePath, Vehicle egoVehicle,
            TraceCaptureConfig config)
            : this(filePath, egoVehicle)
        {
            this.config = config;
        }

        public void Start()
        {
            // difference between ROS time and Unity time
            var rosTime = SimulatorROS2Node.GetCurrentRosTime();
            rosTimeAtStart = rosTime.Sec + rosTime.Nanosec / Math.Pow(10, 9);
            switch (config.TraceCaptureFrom)
            {
                case CaptureStartingTime.AW_LOCALIZATION_INITIALIZED:
                    try
                    {
                        SimulatorROS2Node.CreateSubscription<LocalizationInitializationState>(
                        TopicName.TOPIC_LOCALIZATION_INITIALIZATION_STATE, msg =>
                        {
                            if (msg.State == LocalizationInitializationState.INITIALIZED)
                            {
                                ready = true;
                                timeStart = timeNow;
                                Debug.Log("[AWAnalysis] Start capturing trace");
                                SubscribeObjectDetectionMsg();
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
                    ready = true;
                    timeStart = 0;
                    SubscribeObjectDetectionMsg();
                    break;
            }
        }

        public void Update()
        {
            timeNow = Time.time;
            if (!ready || fileWritten)
                return;
            if (timeNow - timeStart >= ConfigLoader.Config().captureLength && !fileWritten)
            {
                while (objectDetectedMsgs.Count > 0)
                {
                    var tuple = objectDetectedMsgs.Dequeue();
                    string objectsStr = "";
                    foreach (var detectedObject in tuple.Item2)
                    {
                        objectsStr += ", " + DumpDetectedObject(detectedObject);
                    }
                    if (Math.Abs(tuple.Item1 - preLastGroundTruthState.Item1) <=
                        lastGroundTruthState.Item1 - tuple.Item1)
                    {
                        preLastGroundTruthState = new Tuple<double, string>(
                            preLastGroundTruthState.Item1,
                            preLastGroundTruthState.Item2 + objectsStr);
                    }
                    else
                    {
                        lastGroundTruthState = new Tuple<double, string>(
                            lastGroundTruthState.Item1,
                            lastGroundTruthState.Item2 + objectsStr);
                    }
                } 
                
                contents += $"{preLastGroundTruthState.Item2}}} .\n  rl {preLastGroundTruthState.Item2}}}\n  => ";
                contents += $"{lastGroundTruthState.Item2}}} .\nendm";
                // add comments
                contents += $"\n--- auto mode ready time: {autoOpModeReadTime}";
                File.WriteAllText(filePath, contents);
                fileWritten = true;
                return;
            }

            double timeStamp = timeNow + rosTimeAtStart;
            string stateStr = $"{timeStamp} # {{";
            
            // Dump ground truth trace of Ego and NPCs
            stateStr += DumpEgoInfo();

            while (objectDetectedMsgs.Count > 0)
            {
                var tuple = objectDetectedMsgs.Peek();
                if (lastGroundTruthState == null ||
                     preLastGroundTruthState == null ||
                     tuple.Item1 > lastGroundTruthState.Item1)
                    break;
                string objectsStr = "";
                foreach (var dynamicObject in tuple.Item2)
                {
                    objectsStr += ", " + DumpDetectedObject(dynamicObject);
                }
                if (Math.Abs(tuple.Item1 - preLastGroundTruthState.Item1) <=
                    lastGroundTruthState.Item1 - tuple.Item1)
                {
                    preLastGroundTruthState = new Tuple<double, string>(
                        preLastGroundTruthState.Item1,
                        preLastGroundTruthState.Item2 + objectsStr);
                }
                else
                {
                    lastGroundTruthState = new Tuple<double, string>(
                        lastGroundTruthState.Item1,
                        lastGroundTruthState.Item2 + objectsStr);
                }
                objectDetectedMsgs.Dequeue();
            } 

            List<NPCVehicle> npcs = CustomNPCSpawningManager.GetNPCs();
            npcs.ForEach(npc => stateStr += ", " + DumpNPCInfo(npc));
            // A closing bracket is missing.
            // stateStr += "}";

            if (preLastGroundTruthState == null)
                preLastGroundTruthState = new Tuple<double, string>(timeStamp, stateStr);
            else if (lastGroundTruthState == null)
                lastGroundTruthState = new Tuple<double, string>(timeStamp, stateStr);
            else
            {
                // don't forget to add a closing bracket
                contents += $"{preLastGroundTruthState.Item2}}} .\n  rl {preLastGroundTruthState.Item2}}}\n  => ";
                preLastGroundTruthState = lastGroundTruthState;
                lastGroundTruthState = new Tuple<double, string>(timeStamp, stateStr);
            }
        }
        
        private void SubscribeObjectDetectionMsg()
        {
            SimulatorROS2Node.CreateSubscription<PredictedObjects>(
                TopicName.TOPIC_PERCEPTION_RECOGNITION_OBJECTS, msg =>
                {
                    if (msg.Objects.Length > 0)
                        objectDetectedMsgs.Enqueue(new Tuple<double, PredictedObject[]>(
                            msg.Header.Stamp.Sec + msg.Header.Stamp.Nanosec / Math.Pow(10, 9),
                            msg.Objects));
                });
            
            // log the time when auto ready
            SimulatorROS2Node.CreateSubscription<OperationModeState>(
                TopicName.TOPIC_API_OPERATION_MODE_STATE, msg =>
                {
                    if (autoOpModeReadTime < 0)
                        autoOpModeReadTime = timeNow;
                });
        }

        private string DumpEgoInfo()
        {
            string pose = $"pose: {{pos: {egoVehicle.Position.x} {egoVehicle.Position.y} {egoVehicle.Position.z}, qua: {egoVehicle.Rotation.x} {egoVehicle.Rotation.y} {egoVehicle.Rotation.z} {egoVehicle.Rotation.w}}}";
            string twist = $"twist: {{lin: {egoVehicle.Velocity.x} {egoVehicle.Velocity.y} {egoVehicle.Velocity.z}, ang: {egoVehicle.AngularVelocity.x} {egoVehicle.AngularVelocity.y} {egoVehicle.AngularVelocity.z}}}";
            string accel = $"accel: {{lin: {egoVehicle.Acceleration.x} {egoVehicle.Acceleration.y} {egoVehicle.Acceleration.z}, ang: {egoVehicle.AngularAcceleration.x} {egoVehicle.AngularAcceleration.y} {egoVehicle.AngularAcceleration.z}}}";
            return $"{{name: \"ego\", {pose}, {twist}, {accel}}}";
        }

        private string DumpNPCInfo(NPCVehicle npc)
        {
            string pose = $"pose: {{pos: {npc.Position.x} {npc.Position.y} {npc.Position.z}, rota: {npc.EulerAnguleY}}}";
            string twist = $"twist: {{lin: {npc.Velocity.x} {npc.Velocity.y} {npc.Velocity.z}, ang: {npc.YawAngularSpeed}}}";
            string accel = $"accel: {npc.Acceleration}";
            return $"{{name: \"{npc.ScriptName ?? ""}\", {pose}, {twist}, {accel}}}";
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
    }
}