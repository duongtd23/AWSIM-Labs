using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.TrafficSimulation;
using System;

namespace AWSIM
{
    public class PerceptionAnalysis : MonoBehaviour
    {
        private NPCVehicle npc;
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
                                (float)roi.Y_offset,
                                (float)roi.Width,
                                (float)roi.Height);
                            Debug.Log("[AWAnalyzer] Detected object " + msg.Feature_objects[i].Object.Classification + boundingBox);
                        }
                    });
            }
            catch (NullReferenceException e)
            {
                Debug.LogError("[AWAnalyzer] Cannot create ROS subscriber /perception/object_recognition/detection/rois0. " +
                    "Make sure Autoware has been started. Exception detail: " + e);
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            if (npc == null && CustomNPCSpawningManager.Manager() != null)
                ToySpawn();
            if (npc != null)
            {
                AWSIMBoundingBox();
                //Test2();
            }
        }
        void ToySpawn()
        {
            LanePosition spawnPosition = new LanePosition("TrafficLane.240", 15f);
            npc = CustomNPCSpawningManager.SpawnNPC("taxi", spawnPosition, out int waypointIndex);
        }
        private Rect AWSIMBoundingBox()
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
            Debug.Log("[AWAnalyzer] Bounding box: " + boundingBox);
            return boundingBox;
        }

        void Test2()
        {
            Mesh mesh = npc.gameObject.GetComponentsInChildren<MeshFilter>()[6].mesh;
            Bounds localBounds = npc.Bounds;
            Vector3[] localVertices = mesh.vertices;

            var worldVertices = new List<Vector3>();
            for (int i = 0; i < localVertices.Length; i++)
                if (localBounds.Contains(localVertices[i]))
                    worldVertices.Add(npc.transform.TransformPoint(localVertices[i]));

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

            //Vector2[] vertices_2d = new Vector2[localVertices.Length];
            //for (var i = 0; i < localVertices.Length; i++)
            //    vertices_2d[i] = sensorCamera.WorldToScreenPoint(
            //        npc.transform.TransformPoint(localVertices[i]));

            //// find the min max bounds of the 2D points
            //Vector2 min = vertices_2d[0];
            //Vector2 max = vertices_2d[0];
            //foreach (Vector2 vertex in vertices_2d)
            //{
            //    min = Vector2.Min(min, vertex);
            //    max = Vector2.Max(max, vertex);
            //}
            //Rect boundingBox = Rect.MinMaxRect(min.x, min.y, max.x, max.y);

            Debug.Log("[AWAnalyzer] Bounding box 2: " + boundingBox);
            //GUI.Box(new Rect(0, 0, 100, 100), "screenRect");
        }

        //private void OnGUI()
        //{
        //    //Test();
        //    GUI.Box(new Rect(1103, 246, 443, 279), "screenRect");
        //}

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
    }
}