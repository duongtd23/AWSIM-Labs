using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.TrafficSimulation;
using System;

namespace AWSIM
{
    public class PerceptionAnalysis : MonoBehaviour
    {
        private NPCVehicle dummy;
        public Camera sensorCamera;

        // Start is called before the first frame update
        void Start()
        {
            try
            {
                SimulatorROS2Node.CreateSubscription<tier4_perception_msgs.msg.DetectedObjectsWithFeature>(
                    "/perception/object_recognition/detection/rois0", msg =>
                    {
                        for (int i = 0; i < msg.Feature_objects.Length; i++)
                        {
                            var roi = msg.Feature_objects[i].Feature.Roi;
                            Rect boundingBox = new Rect(
                                (float)roi.X_offset,
                                (float)sensorCamera.pixelHeight - roi.Y_offset - roi.Height,
                                (float)roi.Width,
                                (float)roi.Height);
                            Debug.Log("[AWAnalysis] Detected object " + msg.Feature_objects[i].Object.Classification + boundingBox);
                        }
                    });
            }
            catch (NullReferenceException e)
            {
                Debug.LogError("[AWAnalysis] Cannot create ROS subscriber /perception/object_recognition/detection/rois0. " +
                    "Make sure Autoware has been started. Exception detail: " + e);
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            if (dummy == null && CustomNPCSpawningManager.Manager() != null)
                ToySpawn();
            if (dummy != null)
            {
                //AutowareAnalysisUtils.Screenshot(sensorCamera, "screenshot.png");
                AWSIMBoundingBox(dummy);
            }
        }
        void ToySpawn()
        {
            LanePosition spawnPosition = new LanePosition("TrafficLane.240", 15f);
            dummy = CustomNPCSpawningManager.SpawnNPC("taxi", spawnPosition, out int waypointIndex);
        }

        /// <summary>
        /// return the bounding box of `npc`.
        /// The returned Rect is tight (more precise than the one returned by AWSIMBoundingBox2)
        /// </summary>
        /// <param name="npc"></param>
        /// <returns></returns>
        private Rect AWSIMBoundingBox(NPCVehicle npc)
        {
            MeshCollider bodyCollider = AutowareAnalysisUtils.GetNPCMeshCollider(npc);
            Vector3 localPosition = bodyCollider.transform.parent.localPosition;

            Mesh mesh = bodyCollider.sharedMesh;
            //DrawMesh(npc, mesh, localPosition);
            Vector3[] localVertices = mesh.vertices;

            var worldVertices = new List<Vector3>();
            for (int i = 0; i < localVertices.Length; i++)
                worldVertices.Add(npc.transform.TransformPoint(
                    localVertices[i] + localPosition));

            var screenVertices = new Vector3[worldVertices.Count];
            for (int i = 0; i < worldVertices.Count; i++)
                screenVertices[i] = sensorCamera.WorldToScreenPoint(worldVertices[i]);

            float min_x = screenVertices[0].x;
            float min_y = screenVertices[0].y;
            float max_x = screenVertices[0].x;
            float max_y = screenVertices[0].y;

            for (int i = 1; i < screenVertices.Length; i++)
            {
                if (screenVertices[i].x < min_x)
                    min_x = screenVertices[i].x;
                if (screenVertices[i].y < min_y)
                    min_y = screenVertices[i].y;
                if (screenVertices[i].x > max_x)
                    max_x = screenVertices[i].x;
                if (screenVertices[i].y > max_y)
                    max_y = screenVertices[i].y;
            }
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
            Bounds localBounds = npc.Bounds;
            var localCorners = new Vector3[8];
            localCorners[0] = new Vector3(localBounds.center.x - localBounds.extents.x, localBounds.center.y - localBounds.extents.y, localBounds.center.z - localBounds.extents.z);
            localCorners[1] = new Vector3(localBounds.center.x + localBounds.extents.x, localBounds.center.y - localBounds.extents.y, localBounds.center.z - localBounds.extents.z);
            localCorners[2] = new Vector3(localBounds.center.x + localBounds.extents.x, localBounds.center.y - localBounds.extents.y, localBounds.center.z + localBounds.extents.z);
            localCorners[3] = new Vector3(localBounds.center.x - localBounds.extents.x, localBounds.center.y - localBounds.extents.y, localBounds.center.z + localBounds.extents.z);
            localCorners[4] = new Vector3(localBounds.center.x - localBounds.extents.x, localBounds.center.y + localBounds.extents.y, localBounds.center.z - localBounds.extents.z);
            localCorners[5] = new Vector3(localBounds.center.x + localBounds.extents.x, localBounds.center.y + localBounds.extents.y, localBounds.center.z - localBounds.extents.z);
            localCorners[6] = new Vector3(localBounds.center.x + localBounds.extents.x, localBounds.center.y + localBounds.extents.y, localBounds.center.z + localBounds.extents.z);
            localCorners[7] = new Vector3(localBounds.center.x - localBounds.extents.x, localBounds.center.y + localBounds.extents.y, localBounds.center.z + localBounds.extents.z);

            var worldCorners = new Vector3[8];
            for (int i = 0; i < 8; i++)
                worldCorners[i] = npc.transform.TransformPoint(localCorners[i]);

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