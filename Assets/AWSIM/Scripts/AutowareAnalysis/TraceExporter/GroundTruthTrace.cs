using System;
using System.Collections.Generic;
using System.IO;
using autoware_adapi_v1_msgs.msg;
using AWSIM.AWAnalysis.CustomSim;
using Unity.Mathematics;
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
        private TraceCaptureConfig config;
        private bool ready, fileWritten;
        private float timeStart;
        private float timeNow;

        // ROS time at start
        private double rosTimeAtStart;

        public const int CAPTURE_DURATION = 60;

        public GroundTruthTrace(string filePath, GameObject autowareEgoCar)
        {
            this.filePath = filePath;
            this.autowareEgoCar = autowareEgoCar;
            this.config = new TraceCaptureConfig(CaptureStartingTime.AW_LOCALIZATION_INITIALIZED);
            contents = TEMPLATE;
        }

        public GroundTruthTrace(string filePath, GameObject autowareEgoCar,
            TraceCaptureConfig config)
            : this(filePath, autowareEgoCar)
        {
            this.config = config;
        }

        public void Start()
        {
            // difference between ROS time and Unity time
            var rosTime = SimulatorROS2Node.GetCurrentRosTime();
            var unityTime = Time.fixedTime;
            rosTimeAtStart = rosTime.Sec + rosTime.Nanosec / Math.Pow(10, 9) - unityTime;
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
                contents += "terminate .\nendm";
                File.WriteAllText(filePath, contents);
                fileWritten = true;
                return;
            }

            string stateStr = $"{timeNow + rosTimeAtStart} # {{";
            stateStr += DumpEgoInfo();

            // while (CustomNPCSpawningManager.Manager() == null) ;
            List<NPCVehicle> npcs = CustomNPCSpawningManager.GetNPCs();
            npcs.ForEach(npc => stateStr += ", " + DumpNPCInfo(npc));
            stateStr += "}";

            contents += stateStr + " .";
            contents += $"\n  rl {stateStr}\n  => ";
        }

        private string DumpEgoInfo()
        {
            Transform tf = autowareEgoCar.GetComponentAtIndex<Transform>(0);
            string pose = $"pose: {{pos: {tf.position.x} {tf.position.y} {tf.position.z}, qua: {tf.rotation.x} {tf.rotation.y} {tf.rotation.z} {tf.rotation.w}}}";

            Rigidbody rb = autowareEgoCar.GetComponentAtIndex<Rigidbody>(1);
            string twist = $"twist: {{lin: {rb.velocity.x} {rb.velocity.y} {rb.velocity.z}, ang: {rb.angularVelocity.x} {rb.angularVelocity.y} {rb.angularVelocity.z}}}";
            return $"{{name: \"ego\", {pose}, {twist}, accel: nilTwist}}";
        }

        private string DumpNPCInfo(NPCVehicle npc)
        {
            string pose = $"pose: {{pos: {npc.Position.x} {npc.Position.y} {npc.Position.z}, rota: {npc.EulerAnguleY}}}";
            string twist = $"twist: {{lin: {npc.Velocity.x} {npc.Velocity.y} {npc.Velocity.z}, ang: {npc.YawAngularSpeed}}}";
            string accel = $"accel: {npc.Acceleration}";
            return $"{{id: [{npc.VehicleID}], name: \"{npc.ScriptName ?? ""}\", {pose}, {twist}, {accel}}}";
        }
    }
}