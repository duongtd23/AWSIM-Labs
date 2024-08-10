using System;
using System.Collections.Generic;
using System.IO;
using autoware_adapi_v1_msgs.msg;
using AWSIM.AWAnalysis.CustomSim;
using UnityEngine;

namespace AWSIM.AWAnalysis.TraceExporter
{
	public class GroundTruthTrace
	{
        public const string TAB = "  ";
        public const string TEMPLATE = "in base.maude\n\nmod TRACE is " +
            "\n  pr AWSTATE .\n\n  eq init = ";
        
        private string contents;
        private string filePath;
        private GameObject autowareEgoCar;
        private string lastObjStr;
		private TraceCaptureConfig config;
		private bool ready, fileWritten;
        private float timeStart;
        private float timeNow;

        public const int CAPTURE_DURATION = 60;

        public GroundTruthTrace(string filePath, GameObject autowareEgoCar)
		{
			this.filePath = filePath;
            this.autowareEgoCar = autowareEgoCar;
			this.config = new TraceCaptureConfig(CaptureStartingTime.AW_LOCALIZATION_INITIALIZED);
            contents = TEMPLATE;
            lastObjStr = "";
        }

        public GroundTruthTrace(string filePath, GameObject autowareEgoCar,
            TraceCaptureConfig config)
            :this(filePath,autowareEgoCar)
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
                            Debug.Log("[AWAnalysis] Start capturing ground truth trace");
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
            timeNow = Time.fixedTime;
            if (!ready || fileWritten)
                return;
            if (Time.fixedTime - timeStart >= CAPTURE_DURATION && !fileWritten)
            {
                contents += "\n" + TAB +
                    "rl " + lastObjStr + "\n" + TAB +
                    "=> terminate .";
                contents += "\nendm";
                File.WriteAllText(filePath, contents);
                fileWritten = true;
                return;
            }
            while (CustomNPCSpawningManager.Manager() == null) ;
            List<NPCVehicle> npcs = CustomNPCSpawningManager.GetNPCs();
            var rosTime = SimulatorROS2Node.GetCurrentRosTime();
            string stateStr = "time(" + rosTime.Sec + ", " + rosTime.Nanosec + ") # {";
            stateStr += DumpEgoInfo();
            npcs.ForEach(npc => stateStr += ", " + DumpNPCInfo(npc));
            stateStr += "}";

            if (lastObjStr == "")
                contents += stateStr + " .";
            else
                contents += "\n" + TAB +
                    "rl " + lastObjStr + "\n" + TAB + "=> " +
                    stateStr + " .";
            lastObjStr = stateStr;

            contents += stateStr;
        }

        private string DumpEgoInfo()
        {
            string id = "id: nil";
            string name = "name: \"ego\"";
            string pose = DumpPoseInfo(autowareEgoCar.transform);

            var rigibody = autowareEgoCar.GetComponent<Rigidbody>();
            string twist = DumpTwistInfo(rigibody);
            string accel = "accel: nilTwist";
            return "{" + id + ", " + name + ", " + pose + ", " + twist + ", " + accel + "}";
        }

        private string DumpNPCInfo(NPCVehicle npc)
        {
            byte[] bytes = BitConverter.GetBytes(npc.VehicleID);
            string id = "id:";
            foreach (byte b in bytes)
            {
                int temp = b;
                id += " " + temp;
            }
            //for (int i = bytes.Length; i < 32; i++)
            string name = "name: \"" + npc.name + "\"";
            string pose = DumpPoseInfo(npc.RigidBodyTransform);

            string twist = DumpTwistInfo(npc);
            string accel = "accel: nilTwist";
            return "{" + id + ", " + name + ", " + pose + ", " + twist + ", " + accel + "}";
        }

        // pose = position + rotation
        private string DumpPoseInfo(Transform transform)
        {
            string pose = "pose: {pos: ";
            Vector3 position = transform.position;
            pose += PerceptionTrace.DoubleToMaudeString(position.x) + " ";
            pose += PerceptionTrace.DoubleToMaudeString(position.y) + " ";
            pose += PerceptionTrace.DoubleToMaudeString(position.z) + ", qua: ";
            // TODO: the other work may compute the difference of rotation based on
            // its x,y,z, and w components,
            // this is not resonable since even a slight change of one component,
            // the rotation might become totally different
            // NEED TO CHECK LATER
            Quaternion rotation = transform.rotation;
            pose += PerceptionTrace.DoubleToMaudeString(rotation.x) + " ";
            pose += PerceptionTrace.DoubleToMaudeString(rotation.y) + " ";
            pose += PerceptionTrace.DoubleToMaudeString(rotation.z) + " ";
            pose += PerceptionTrace.DoubleToMaudeString(rotation.w) + "}";
            return pose;
        }

        // twist = linear velocity + angular velocity
        private string DumpTwistInfo(NPCVehicle npc)
        {
            Vector3 linearVel = npc.Velocity;
            string twist = "twist: {lin: ";
            twist += PerceptionTrace.DoubleToMaudeString(linearVel.x) + " ";
            twist += PerceptionTrace.DoubleToMaudeString(linearVel.y) + " ";
            twist += PerceptionTrace.DoubleToMaudeString(linearVel.z) + ", ang: ";
            twist += "0.0 ";
            twist += PerceptionTrace.DoubleToMaudeString(npc.YawAngularSpeed) + " ";
            twist += "0.0}";
            return twist;
        }

        private string DumpTwistInfo(Rigidbody rigibody)
        {
            Vector3 linearVel = rigibody.velocity;
            Vector3 angularVel = rigibody.angularVelocity;
            string twist = "twist: {lin: ";
            twist += PerceptionTrace.DoubleToMaudeString(linearVel.x) + " ";
            twist += PerceptionTrace.DoubleToMaudeString(linearVel.y) + " ";
            twist += PerceptionTrace.DoubleToMaudeString(linearVel.z) + ", ang: ";
            twist += PerceptionTrace.DoubleToMaudeString(angularVel.x) + " ";
            twist += PerceptionTrace.DoubleToMaudeString(angularVel.y) + " ";
            twist += PerceptionTrace.DoubleToMaudeString(angularVel.z) + "}";
            return twist;
        }
    }
}