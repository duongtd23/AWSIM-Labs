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
        private string lastObjStr;
        private TraceCaptureConfig config;
        private bool ready, fileWritten;
        private float timeStart;

        public const int CAPTURE_DURATION = 60;

        public PerceptionTrace(string filePath)
        {
            this.filePath = filePath;
            config = new TraceCaptureConfig(CaptureStartingTime.AW_LOCALIZATION_INITIALIZED);
            contents = GroundTruthTrace.TEMPLATE;
            lastObjStr = "";
        }
        public PerceptionTrace(string filePath, TraceCaptureConfig config)
            :this(filePath)
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
                        "/api/localization/intialization_state", msg =>
                        {
                        if (msg.State == LocalizationInitializationState.INITIALIZED)
                        {    
                            ready = true;
                            timeStart = Time.fixedTime;
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

        public void FixedUpdate()
        {
            if (!ready || fileWritten)
                return;
            if (Time.fixedTime - timeStart >= CAPTURE_DURATION && !fileWritten)
            {
                contents += "\n" + GroundTruthTrace.TAB +
                    "rl " + lastObjStr + "\n" + GroundTruthTrace.TAB +
                    "=> terminate .";
                contents += "\nendm";
                File.WriteAllText(filePath, contents);
                fileWritten = true;
                return;
            }
        }

        private void HandleDetectedObjectsMsg(DynamicObjectArray msg)
        {
            string msg2Str = WriteTimeStamp(msg.Header.Stamp);
            msg2Str += " # ";
            if (msg.Objects.Length < 1)
                return;

            msg2Str += WriteObjects(msg.Objects);

            if (lastObjStr == "")
            {
                contents += msg2Str + " .";
            }
            else
            {
                contents += "\n" + GroundTruthTrace.TAB +
                    "rl " + lastObjStr + "\n" + GroundTruthTrace.TAB + "=> " +
                    msg2Str + " .";
            }
            lastObjStr = msg2Str;
        }

        private string WriteTimeStamp(builtin_interfaces.msg.Time time)
        {
            return "time(" + time.Sec + ", " + time.Nanosec + ")";
        }

        private string WriteObjects(DynamicObject[] objects)
        {
            string result = WriteObject(objects[0]);
            for (int i = 1; i < objects.Length; i++)
            {
                result += ", " + WriteObject(objects[i]);
            }
            return "{" + result + "}";
        }

        private string WriteObject(DynamicObject obj)
        {
            string id = "id:";
            for (int i = 0; i < obj.Id.Uuid.Length; i++)
            {
                int temp = obj.Id.Uuid[i];
                id += " " + temp;
            }
            string name = "name: nil";
            string pose = "pose: " + PoseToString(obj.Kinematics.Pose);
            string twist = "twist: " + TwistToString(obj.Kinematics.Twist);
            string accel = "accel: " + AccelToString(obj.Kinematics.Accel);

            return "{" + id + ", " + name + ", " + pose + ", " + twist + ", " + accel + "}";
        }

        private string PoseToString(geometry_msgs.msg.Pose pose)
        {
            string result = "{pos: ";
            result += DoubleToMaudeString(pose.Position.X) + " ";
            result += DoubleToMaudeString(pose.Position.Y) + " ";
            result += DoubleToMaudeString(pose.Position.Z) + ", qua: ";
            result += DoubleToMaudeString(pose.Orientation.X) + " ";
            result += DoubleToMaudeString(pose.Orientation.Y) + " ";
            result += DoubleToMaudeString(pose.Orientation.Z) + " ";
            result += DoubleToMaudeString(pose.Orientation.W) + "}";
            return result;
        }

        private string TwistToString(geometry_msgs.msg.Twist twist)
        {
            string result = "{lin: ";
            result += DoubleToMaudeString(twist.Linear.X) + " ";
            result += DoubleToMaudeString(twist.Linear.Y) + " ";
            result += DoubleToMaudeString(twist.Linear.Z) + ", ang: ";
            result += DoubleToMaudeString(twist.Angular.X) + " ";
            result += DoubleToMaudeString(twist.Angular.Y) + " ";
            result += DoubleToMaudeString(twist.Angular.Z) + "}";
            return result;
        }

        private string AccelToString(geometry_msgs.msg.Accel twist)
        {
            string result = "{lin: ";
            result += DoubleToMaudeString(twist.Linear.X) + " ";
            result += DoubleToMaudeString(twist.Linear.Y) + " ";
            result += DoubleToMaudeString(twist.Linear.Z) + ", ang: ";
            result += DoubleToMaudeString(twist.Angular.X) + " ";
            result += DoubleToMaudeString(twist.Angular.Y) + " ";
            result += DoubleToMaudeString(twist.Angular.Z) + "}";
            return result;
        }

        public static string DoubleToMaudeString(double number)
        {
            string str = number.ToString();
            if (!str.Contains("."))
                str += ".0";
            return str;
        }
    }
}
