using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.AWAnalysis.CustomSim;
using System;
using tier4_perception_msgs.msg;
using AWSIM.AWAnalysis.TraceExporter;

namespace AWSIM.AWAnalysis
{
    public class PerceptionAnalysis : MonoBehaviour
    {
        public Camera sensorCamera;
        public GameObject autowareEgoCar;

        // ignore NPCs outside of this distance
        public const int NPC_RECOGNIZE_DISTANCE = 130;

        // the last ROS2 message received
        private DetectedObjectsWithFeature lastBoundingBoxMsgReceived;

        private autoware_adapi_v1_msgs.msg.DynamicObjectArray lastDetectedObjectsMsgReceived;
        // each 100 milliseconds
        public const int TRACE_RATE = 100;
        private TraceWriter traceWriter;

        private int timeStepCount = 0;

        private bool autowareActive = true;
        private bool perceptionAnalysisEnable = true;

        void Start()
        {
            // parse command line arguments
            CommandLineArgsManager.Instance();

            bool argDefined = CommandLineArgsManager.GetPerceptionAnalysisFlag(out bool flag);
            if (argDefined)
                perceptionAnalysisEnable = flag;
            if (perceptionAnalysisEnable)
            {
                try
                {
                    SimulatorROS2Node.CreateSubscription<DetectedObjectsWithFeature>(
                    "/perception/object_recognition/detection/rois0", msg =>
                    {
                        lastBoundingBoxMsgReceived = msg;
                    });
                    traceWriter = new TraceWriter("trace.maude");
                    SimulatorROS2Node.CreateSubscription<
                        autoware_adapi_v1_msgs.msg.DynamicObjectArray>(
                    "/api/perception/objects", msg =>
                    {
                        HandleDetectedObjectsMsg(msg);
                    });
                }
                catch (NullReferenceException e)
                {
                    autowareActive = false;
                    Debug.LogError("[AWAnalysis] Cannot create ROS subscriber /perception/object_recognition/detection/rois0. " +
                        "Make sure Autoware has been started. Exception detail: " + e);
                }
            }
        }

        void FixedUpdate()
        {
            if (perceptionAnalysisEnable && autowareActive)
            {
                if (Time.fixedTime > 60)
                    traceWriter.WriteFile();
                timeStepCount = (timeStepCount + 1) % 10;
                if (timeStepCount % 10 != 9)
                    return;

                // invoke the following every 0.02*10 second
                if (CustomNPCSpawningManager.Manager() != null &&
                    CustomNPCSpawningManager.GetNPCs() != null)
                {
                    List<Rect> detectedBoxes = ParseDetectedMsg(lastBoundingBoxMsgReceived);
                    foreach (NPCVehicle npc in CustomNPCSpawningManager.GetNPCs())
                    {
                        var distance = Vector3.Distance(
                            npc.transform.position,
                            autowareEgoCar.transform.position);
                        // if the NPC is within the distance considered and
                        // it is visible by the sensor camera view 
                        if (distance < NPC_RECOGNIZE_DISTANCE &&
                            CameraUtils.NPCVisibleByCamera(sensorCamera, npc))
                            EvaluatePerception(npc, detectedBoxes);
                    }
                }
            }
        }

        private void HandleDetectedObjectsMsg(autoware_adapi_v1_msgs.msg.DynamicObjectArray msg)
        {
            float diff = AutowareAnalysisUtils.DiffInMiliSec(msg.Header.Stamp, lastBoundingBoxMsgReceived.Header.Stamp);
            if (diff >= TRACE_RATE)
            {
                traceWriter.AppendMsg(msg);
                lastDetectedObjectsMsgReceived = msg;
            }
        }

        /// <summary>
        /// parse the last received message to get a list of bounding box
        /// </summary>
        /// <param name="msg"></param>
        /// <returns></returns>
        private List<Rect> ParseDetectedMsg(tier4_perception_msgs.msg.DetectedObjectsWithFeature msg)
        {
            List<Rect> boxes = new List<Rect>();
            if (msg == null)
                return boxes;
            for (int i = 0; i < msg.Feature_objects.Length; i++)
            {
                var roi = msg.Feature_objects[i].Feature.Roi;
                Rect detectedBox = new Rect(
                    (float)roi.X_offset,
                    (float)sensorCamera.pixelHeight - roi.Y_offset - roi.Height,
                    (float)roi.Width,
                    (float)roi.Height);
                Debug.Log("[AWAnalysis] Autoware detected an object: " + msg.Feature_objects[i].Object.Classification + detectedBox);
                boxes.Add(detectedBox);
            }
            return boxes;
        }

        /// <summary>
        /// Evaluate perception module
        /// </summary>
        /// <param name="npc">The (ground-trust) NPC vehicle </param>
        /// <param name="detectedBoxes"> the list of bounding boxes detected by Autoware</param>
        private void EvaluatePerception(NPCVehicle npc, List<Rect> detectedBoxes)
        {
            Rect groundTrustBox = AWSIMBoundingBox(npc);
            float bestIOURatio = 0;
            int bestMatchBoxIndex = -1;
            for (int i = 0; i < detectedBoxes.Count; i++)
            {
                float iouRatio = BoundingBoxUtils.IOURatio(detectedBoxes[i], groundTrustBox);
                if (iouRatio > bestIOURatio)
                {
                    bestIOURatio = iouRatio;
                    bestMatchBoxIndex = i;
                }
            }
            if (bestMatchBoxIndex == -1)
            {
                Debug.LogError("[AWAnalysis] No detected box with IOU ratio > 0 found" +
                    " for NPC " + npc.name + " whose bounding box is " + groundTrustBox);
            }
            else
            {
                Debug.Log("[AWAnalysis] Found a detected bounding box " +
                    detectedBoxes[bestMatchBoxIndex]  + " matches with IOU ratio " +
                    bestIOURatio * 100 + "% " +
                    "for NPC " + npc.name + " whose bounding box is " + groundTrustBox);
            }
        }

        /// <summary>
        /// return the bounding box of `npc`.
        /// The returned Rect is tight (more precise than the one returned by AWSIMBoundingBox2)
        /// </summary>
        /// <param name="npc"></param>
        /// <returns></returns>
        private Rect AWSIMBoundingBox(NPCVehicle npc)
        {
            MeshCollider bodyCollider = CameraUtils.GetNPCMeshCollider(npc);
            Vector3 localPosition = bodyCollider.transform.parent.localPosition;

            Mesh mesh = bodyCollider.sharedMesh;
            //DrawMesh(npc, mesh, localPosition);
            Vector3[] localVertices = mesh.vertices;

            var worldVertices = new List<Vector3>();
            for (int i = 0; i < localVertices.Length; i++)
                worldVertices.Add(npc.transform.TransformPoint(
                    localVertices[i] + localPosition));

            var screenVertices = new List<Vector3>();
            for (int i = 0; i < worldVertices.Count; i++)
            {
                var screenPoint = sensorCamera.WorldToScreenPoint(worldVertices[i]);
                if (screenPoint.z > 0)
                {
                    screenPoint = CameraUtils.FixScreenPoint(screenPoint, sensorCamera);
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
                    CameraUtils.InRange(screenVertices[i].y, 0, sensorCamera.pixelHeight))
                    min_x = screenVertices[i].x;
                if (screenVertices[i].y < min_y &&
                    CameraUtils.InRange(screenVertices[i].x, 0, sensorCamera.pixelWidth))
                    min_y = screenVertices[i].y;
                if (screenVertices[i].x > max_x &&
                    CameraUtils.InRange(screenVertices[i].y, 0, sensorCamera.pixelHeight))
                    max_x = screenVertices[i].x;
                if (screenVertices[i].y > max_y &&
                    CameraUtils.InRange(screenVertices[i].x, 0, sensorCamera.pixelWidth))
                    max_y = screenVertices[i].y;
            }
            // if min_x is -0.1
            min_x = Mathf.Max(0, min_x);
            min_y = Mathf.Max(0, min_y);
            max_x = Mathf.Min(sensorCamera.pixelWidth, max_x);
            max_y = Mathf.Min(sensorCamera.pixelHeight, max_y);

            Rect boundingBox = Rect.MinMaxRect(min_x, min_y, max_x, max_y);
            Debug.Log("[AWAnalysis] Bounding box of NPC " + npc.name + " : " + boundingBox);
            return boundingBox;
        }

        /// <summary>
        /// return the bounding box of `npc`.
        /// The returned Rect is less precise then the one returned by AWSIMBoundingBox,
        /// but the computation is simpler.
        /// </summary>
        /// <returns></returns>
        private Rect AWSIMBoundingBox2(NPCVehicle npc)
        {
            var worldCorners = CameraUtils.NPCLocalBoundsToWorldCorners(npc);

            //DrawBounds(worldCorners, 0.5f);

            var screenCorners = new Vector3[8];
            for (int i = 0; i < 8; i++)
                screenCorners[i] = sensorCamera.WorldToScreenPoint(worldCorners[i]);
            float min_x = screenCorners[0].x;
            float min_y = screenCorners[0].y;
            float max_x = screenCorners[0].x;
            float max_y = screenCorners[0].y;

            for (int i = 1; i < 8; i++)
            {
                if (screenCorners[i].x < min_x)
                    min_x = screenCorners[i].x;
                if (screenCorners[i].y < min_y)
                    min_y = screenCorners[i].y;
                if (screenCorners[i].x > max_x)
                    max_x = screenCorners[i].x;
                if (screenCorners[i].y > max_y)
                    max_y = screenCorners[i].y;
            }

            Rect boundingBox = Rect.MinMaxRect(min_x, min_y, max_x, max_y);
            Debug.Log("[AWAnalysis] Bounding box of " + npc.name + " :" + boundingBox);
            return boundingBox;
        }

        // for debug purpose
        void DrawBounds(Bounds b, float delay = 0)
        {
            // bottom corners
            var p1 = new Vector3(b.min.x, b.min.y, b.min.z);
            var p2 = new Vector3(b.max.x, b.min.y, b.min.z);
            var p3 = new Vector3(b.max.x, b.min.y, b.max.z);
            var p4 = new Vector3(b.min.x, b.min.y, b.max.z);

            // top corners
            var p5 = new Vector3(b.min.x, b.max.y, b.min.z);
            var p6 = new Vector3(b.max.x, b.max.y, b.min.z);
            var p7 = new Vector3(b.max.x, b.max.y, b.max.z);
            var p8 = new Vector3(b.min.x, b.max.y, b.max.z);
            DrawBounds(new Vector3[8] { p1, p2, p3, p4, p5, p6, p7, p8 }, delay);
        }
        // for debug purpose
        void DrawBounds(Vector3[] corners, float duration = 0)
        {
            // bottom
            Debug.DrawLine(corners[0], corners[1], Color.blue, duration);
            Debug.DrawLine(corners[1], corners[2], Color.red, duration);
            Debug.DrawLine(corners[2], corners[3], Color.yellow, duration);
            Debug.DrawLine(corners[3], corners[0], Color.magenta, duration);

            // top
            Debug.DrawLine(corners[4], corners[5], Color.blue, duration);
            Debug.DrawLine(corners[5], corners[6], Color.red, duration);
            Debug.DrawLine(corners[6], corners[7], Color.yellow, duration);
            Debug.DrawLine(corners[7], corners[4], Color.magenta, duration);

            // sides
            Debug.DrawLine(corners[0], corners[4], Color.white, duration);
            Debug.DrawLine(corners[1], corners[5], Color.gray, duration);
            Debug.DrawLine(corners[2], corners[6], Color.green, duration);
            Debug.DrawLine(corners[3], corners[7], Color.cyan, duration);
        }
        // for debug purpose
        void DrawMesh(NPCVehicle npc, Mesh mesh, Vector3 localPosition)
        {
            var color = Color.red;
            var duration = 0.5f;
            var localVertices = mesh.vertices;
            var worldVertices = new Vector3[localVertices.Length];
            for (int i = 0; i < localVertices.Length; i++)
                worldVertices[i] = npc.transform.TransformPoint(
                    localVertices[i] + localPosition);
            for (int i = 0; i < mesh.triangles.Length / 3; i++)
            {
                Debug.DrawLine(
                    worldVertices[mesh.triangles[i * 3 + 2]],
                    worldVertices[mesh.triangles[i * 3]],
                    color, duration);
                Debug.DrawLine(
                    worldVertices[mesh.triangles[i * 3]],
                    worldVertices[mesh.triangles[i * 3 + 1]],
                    color, duration);
                Debug.DrawLine(
                    worldVertices[mesh.triangles[i * 3 + 1]],
                    worldVertices[mesh.triangles[i * 3 + 2]],
                    color, duration);
            }
        }
    }
}