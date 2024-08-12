using System;
using System.Collections.Generic;
using System.IO;
using autoware_adapi_v1_msgs.msg;
using AWSIM.AWAnalysis.CustomSim;
using UnityEngine;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class PerceptionTrace
    {
        private string contents;
        private string filePath;
        private TraceCaptureConfig config;
        private bool ready, fileWritten;
        private float timeStart;
        private float timeNow;

        public const int CAPTURE_DURATION = 60;

        public PerceptionTrace(string filePath)
        {
            this.filePath = filePath;
            config = new TraceCaptureConfig(CaptureStartingTime.AW_LOCALIZATION_INITIALIZED);
            contents = GroundTruthTrace.TEMPLATE;
        }
        public PerceptionTrace(string filePath, TraceCaptureConfig config)
            : this(filePath)
        {
            this.config = config;
        }

        public void Start()
        {
            switch (config.TraceCaptureFrom)
            {
                case CaptureStartingTime.AW_LOCALIZATION_INITIALIZED:
                    try
                    {
                        SimulatorROS2Node.CreateSubscription<LocalizationInitializationState>(
                        "/localization/initialization_state", msg =>
                        {
                            if (msg.State == LocalizationInitializationState.INITIALIZED)
                            {
                                ready = true;
                                timeStart = timeNow;
                                Debug.Log("[AWAnalysis] Start capturing perception trace");
                                SimulatorROS2Node.CreateSubscription<DynamicObjectArray>(
                                "/api/perception/objects", msg =>
                                {
                                    HandleDetectedObjectsMsg(msg);
                                });
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
                    break;
            }
        }

        public void Update()
        {
            timeNow = Time.time;
            if (!ready || fileWritten)
                return;
            if (timeNow - timeStart >= CAPTURE_DURATION && !fileWritten)
            {
                contents += "terminate .\nendm";
                File.WriteAllText(filePath, contents);
                fileWritten = true;
            }
        }

        private void HandleDetectedObjectsMsg(DynamicObjectArray msg)
        {
            if (msg.Objects.Length < 1)
                return;
            string msg2Str = $"{msg.Header.Stamp.Sec + msg.Header.Stamp.Nanosec / Math.Pow(10, 9)} # {{";
            msg2Str += WriteObject(msg.Objects[0]);
            for (int i = 1; i < msg.Objects.Length; i++)
            {
                msg2Str += ", " + WriteObject(msg.Objects[i]);
            }
            msg2Str += "}";

            contents += msg2Str + " .";
            contents += $"\n  rl {msg2Str}\n  => ";
        }

        private string WriteObject(DynamicObject obj)
        {
            string uuid = "";
            for (int i = 0; i < obj.Id.Uuid.Length; i++)
            {
                uuid += $"{(int)obj.Id.Uuid[i]} ";
            }
            uuid = uuid[..^1];

            string classification = "";
            for (int i = 0; i < obj.Classification.Length; i++)
            {
                classification += $"{(int)obj.Classification[i].Label} -> {obj.Classification[i].Probability}, ";
            }
            classification = classification[..^2];

            var tf = obj.Kinematics.Pose;
            string pose = $"pose: {{pos: {tf.Position.X} {tf.Position.Y} {tf.Position.Z}, qua: {tf.Orientation.X} {tf.Orientation.Y} {tf.Orientation.Z} {tf.Orientation.W}}}";

            var vel = obj.Kinematics.Twist;
            string twist = $"twist: {{lin: {vel.Linear.X} {vel.Linear.Y} {vel.Linear.Z}, ang: {vel.Angular.X} {vel.Angular.Y} {vel.Angular.Z}}}";

            var accel = obj.Kinematics.Accel;
            string accelStr = $"accel: {{lin: {accel.Linear.X} {accel.Linear.Y} {accel.Linear.Z}, ang: {accel.Angular.X} {accel.Angular.Y} {accel.Angular.Z}}}";

            return $"{{id: [{uuid}], epro: {obj.Existence_probability}, class: [{classification}], {pose}, {twist}, {accelStr}}}";
        }
    }
}
