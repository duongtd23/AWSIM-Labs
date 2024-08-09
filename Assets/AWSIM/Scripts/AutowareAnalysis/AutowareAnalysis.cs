using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class AutowareAnalysis : MonoBehaviour
    {
        public GameObject autowareEgoCar;
        private TraceWriter traceWriter;
        private bool traceWritten = false;
        public const int CAPTURE_DURATION = 30;

        // 5 seconds delay before capturing the trace
        private const int DELAY = 5;

        private const int CAPTURE_RATE = 10; // Hz
        private int UPDATE_INTERVAL;
        private int stepCount = 0;

        // Start is called before the first frame update
        void Start()
        {
            UPDATE_INTERVAL = (int)(1 / Time.fixedDeltaTime / CAPTURE_RATE);
            traceWriter = new TraceWriter("groundtrust-trace.maude");
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            if (Time.fixedTime < DELAY)
                return;
            while (CustomNPCSpawningManager.Manager() == null) ;

            // stepCount = (stepCount + 1) % UPDATE_INTERVAL;
            // if (stepCount % UPDATE_INTERVAL != 1)
            //     return;

            if (Time.fixedTime >= CAPTURE_DURATION + DELAY && !traceWritten)
            {
                traceWriter.WriteFile();
                traceWritten = true;
            }
            if (!traceWritten)
            {
                List<NPCVehicle> npcs = CustomNPCSpawningManager.GetNPCs();
                var rosTime = SimulatorROS2Node.GetCurrentRosTime();
                string stateStr = "time(" + rosTime.Sec + ", " + rosTime.Nanosec + ") # {";
                stateStr += DumpEgoInfo();
                npcs.ForEach(npc => stateStr += ", " + DumpNPCInfo(npc));
                stateStr += "}";
                traceWriter.AppendGroundTrustInfo(stateStr);
            }
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
            pose += TraceWriter.DoubleToMaudeString(position.x) + " ";
            pose += TraceWriter.DoubleToMaudeString(position.y) + " ";
            pose += TraceWriter.DoubleToMaudeString(position.z) + ", qua: ";
            // TODO: the other work may compute the difference of rotation based on
            // its x,y,z, and w components,
            // this is not resonable since even a slight change of one component,
            // the rotation might become totally different
            // NEED TO CHECK LATER
            Quaternion rotation = transform.rotation;
            pose += TraceWriter.DoubleToMaudeString(rotation.x) + " ";
            pose += TraceWriter.DoubleToMaudeString(rotation.y) + " ";
            pose += TraceWriter.DoubleToMaudeString(rotation.z) + " ";
            pose += TraceWriter.DoubleToMaudeString(rotation.w) + "}";
            return pose;
        }

        // twist = linear velocity + angular velocity
        private string DumpTwistInfo(NPCVehicle npc)
        {
            Vector3 linearVel = npc.Velocity;
            string twist = "twist: {lin: ";
            twist += TraceWriter.DoubleToMaudeString(linearVel.x) + " ";
            twist += TraceWriter.DoubleToMaudeString(linearVel.y) + " ";
            twist += TraceWriter.DoubleToMaudeString(linearVel.z) + ", ang: ";
            twist += "0.0 ";
            twist += TraceWriter.DoubleToMaudeString(npc.YawAngularSpeed) + " ";
            twist += "0.0}";
            return twist;
        }

        private string DumpTwistInfo(Rigidbody rigibody)
        {
            Vector3 linearVel = rigibody.velocity;
            Vector3 angularVel = rigibody.angularVelocity;
            string twist = "twist: {lin: ";
            twist += TraceWriter.DoubleToMaudeString(linearVel.x) + " ";
            twist += TraceWriter.DoubleToMaudeString(linearVel.y) + " ";
            twist += TraceWriter.DoubleToMaudeString(linearVel.z) + ", ang: ";
            twist += TraceWriter.DoubleToMaudeString(angularVel.x) + " ";
            twist += TraceWriter.DoubleToMaudeString(angularVel.y) + " ";
            twist += TraceWriter.DoubleToMaudeString(angularVel.z) + "}";
            return twist;
        }
    }

}