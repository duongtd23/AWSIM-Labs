using System;
using System.Collections.Generic;
using AWSIM_Script.Object;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class StatusExtraction
    {
        public static EgoGroundTruthObject ExtractEgoKinematic(Vehicle egoVehicle, bool useROSCoord = false)
        {
            var egoPosition = egoVehicle.Position;
            var egoRotation = egoVehicle.Rotation;

            if (useROSCoord)
            {
                egoPosition = ROS2Utility.UnityToRosMGRS(egoPosition);
                egoRotation = ROS2Utility.UnityToRosRotation(egoRotation);
            }

            var result = new EgoGroundTruthObject();
            
            result.pose = new Pose2Object();
            result.pose.position = new Vector3Object(egoPosition);
            result.pose.rotation = new Vector3Object(egoRotation.eulerAngles);
            
            result.twist = new TwistObject();
            result.twist.linear = Vector3ToVector3Object(egoVehicle.Velocity, useROSCoord);
            result.twist.angular = Vector3ToVector3Object(egoVehicle.AngularVelocity, useROSCoord);
            
            result.acceleration = new AccelerationObject();
            result.acceleration.linear = Vector3ToVector3Object(egoVehicle.Acceleration, useROSCoord);
            result.acceleration.angular = Vector3ToVector3Object(egoVehicle.AngularAcceleration, useROSCoord);
            
            return result;
        }

        public static NPCGroundTruthObject[] ExtractNPCKinematics(List<NPCVehicle> npcVehicles, bool useROSCoord = false)
        {
            int npcCount = npcVehicles.Count;
            var result = new NPCGroundTruthObject[npcCount];
            
            for (int i = 0; i < npcCount; i++)
            {
                var npc = npcVehicles[i];
                result[i] = new NPCGroundTruthObject();
                result[i].name = npc.ScriptName;
                
                result[i].pose = new Pose2Object();
                var npcPosition = npc.Position;
                var npcRotationQ = npc.RotationQ;
                if (useROSCoord)
                {
                    npcPosition = ROS2Utility.UnityToRosMGRS(npcPosition);
                    npcRotationQ = ROS2Utility.UnityToRosRotation(npcRotationQ);
                }
                result[i].pose.position = new Vector3Object(npcPosition);
                result[i].pose.rotation = new Vector3Object(npcRotationQ.eulerAngles);
                
                result[i].twist = new TwistObject();
                result[i].twist.linear = Vector3ToVector3Object(npc.Velocity, useROSCoord);
                result[i].twist.angular = Vector3ToVector3Object(new Vector3(0, npc.YawAngularSpeed, 0), useROSCoord);

                result[i].acceleration = npc.Acceleration;
            }
            return result;
        }

        public static PedestrianGtObject[] ExtractPedestrians(List<Tuple<NPCPedes, NPCPedestrian>> npcPedestrians, bool useROSCoord = false)
        {
            var pedesCount = npcPedestrians.Count;
            var result = new PedestrianGtObject[npcPedestrians.Count];
            for (int i = 0; i < pedesCount; i++)
            {
                var entry = npcPedestrians[i];
                result[i] = new PedestrianGtObject();
                result[i].name = entry.Item1.Name;
                
                result[i].pose = new Pose2Object();
                var pedesPosition = entry.Item1.LastPosition;
                var pedesRotation = entry.Item1.LastRotation;
                if (useROSCoord)
                {
                    pedesPosition = ROS2Utility.UnityToRosMGRS(pedesPosition);
                    pedesRotation = ROS2Utility.UnityToRosRotation(pedesRotation);
                }
                result[i].pose.position = new Vector3Object(pedesPosition);
                result[i].pose.rotation = new Vector3Object(pedesRotation.eulerAngles);
                
                result[i].speed = entry.Item1.Config.Speed;
            }
            return result;
        }

        public static BoundingBoxObject[] Extract2DVehicleBoundingBoxes(
            List<NPCVehicle> npcVehicles,
            Camera sensorCamera,
            Vehicle egoVehicle,
            float maxDistanceVisibleOnCamera = 300)
        {
            var result = new BoundingBoxObject[npcVehicles.Count];
            for (int i = 0; i < npcVehicles.Count; i++)
            {
                var npc = npcVehicles[i];
                var distanceToEgo = CustomSimUtils.DistanceIgnoreYAxis(npc.Position, egoVehicle.Position);
                if (distanceToEgo < maxDistanceVisibleOnCamera &&
                    CameraUtils.NPCVisibleByCamera(sensorCamera, npc))
                    result[i] = GetNPCGtBoundingBox(npc, sensorCamera);
            }
            return result;
        }

        public static BoundingBoxObject[] Extract2DPedestrianBoundingBoxes(
            List<Tuple<NPCPedes, NPCPedestrian>> npcPedestrians,
            Camera sensorCamera,
            Vehicle egoVehicle,
            float maxDistanceVisibleOnCamera = 300)
        {
            var result = new BoundingBoxObject[npcPedestrians.Count];
            for (int i = 0; i < npcPedestrians.Count; i++)
            {
                var entry = npcPedestrians[i];
                var distanceToEgo = CustomSimUtils.DistanceIgnoreYAxis(entry.Item1.LastPosition, egoVehicle.Position);
                if (distanceToEgo < maxDistanceVisibleOnCamera &&
                    CameraUtils.PedestrianVisibleByCamera(sensorCamera, entry.Item2))
                    result[i] = GetPedestrianGtBoundingBox(entry.Item2, sensorCamera);
            }
            return result;
        }
        
        /// <summary>
        /// return 2D bounding box of `npc` appear on `sensorCamera` screen
        /// </summary>
        /// <param name="npc"></param>
        /// <returns></returns>
        public static BoundingBoxObject GetNPCGtBoundingBox(NPCVehicle npc, Camera sensorCamera)
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
            return new BoundingBoxObject()
            {
                x = min_x,
                y = min_y,
                width = max_x - min_x,
                height = max_y - min_y
            };
        }
        
        /// <summary>
        /// return the bounding box of `npc` appear on `sensorCamera` screen
        /// </summary>
        /// <param name="pedestrian"></param>
        /// <returns></returns>
        public static BoundingBoxObject GetPedestrianGtBoundingBox(NPCPedestrian pedestrian, Camera sensorCamera)
        {
            var worldVertices = new List<Vector3>();

            // var suitMeshRenderer = pedestrian.GetSuitMeshRenderer();
            // Vector3 suitLocalPosition = suitMeshRenderer.transform.localPosition;
            //
            // Mesh suitMesh = suitMeshRenderer.sharedMesh;
            // Vector3[] suitLocalVertices = suitMesh.vertices;
            //
            // for (int i = 0; i < suitLocalVertices.Length; i+=3)
            //     worldVertices.Add(pedestrian.transform.TransformPoint(
            //         suitLocalVertices[i] + suitLocalPosition));
            
            var shoesMeshRenderer = pedestrian.GetShoesMeshRenderer();
            Vector3 shoesLocalPosition = shoesMeshRenderer.transform.localPosition;
            
            Mesh shoesMesh = shoesMeshRenderer.sharedMesh;
            Vector3[] shoesLocalVertices = shoesMesh.vertices;
            
            for (int i = 0; i < shoesLocalVertices.Length; i+=10)
                worldVertices.Add(pedestrian.transform.TransformPoint(
                    shoesLocalVertices[i] + shoesLocalPosition));
            
            var headMeshRenderer = pedestrian.GetHeadMeshRenderer();
            Vector3 headLocalPosition = headMeshRenderer.transform.localPosition;
            
            Mesh headMesh = headMeshRenderer.sharedMesh;
            Vector3[] headLocalVertices = headMesh.vertices;
            
            for (int i = 0; i < headLocalVertices.Length; i+=10)
                worldVertices.Add(pedestrian.transform.TransformPoint(
                    headLocalVertices[i] + headLocalPosition));

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

            if (screenVertices.Count == 0)
                return new BoundingBoxObject();
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
            return new BoundingBoxObject()
            {
                x = min_x,
                y = min_y,
                width = max_x - min_x,
                height = max_y - min_y
            };
        }

        public static aw_monitor.msg.VehicleSize GetEgoSize(GameObject egoObj, bool useROSCoord = false)
        {
            // ego details
            MeshFilter meshFilter = egoObj.GetComponentInChildren<MeshFilter>();
            var egoCenter = (meshFilter.mesh.bounds.center + meshFilter.transform.parent.parent.localPosition);
            var egoSize = meshFilter.mesh.bounds.extents * 2;
            if (useROSCoord)
            {
                var temp = egoSize.x;
                egoSize.x = egoSize.z;
                egoSize.z = egoSize.y;
                egoSize.y = temp;
            }
            return new aw_monitor.msg.VehicleSize()
            {
                Name = "ego",
                Center = ToROSVector3(egoCenter, useROSCoord),
                Size = ToROSVector3(egoSize)
            };
        }

        public static aw_monitor.msg.VehicleSize GetNPCVehicleSize(NPCVehicle npc, bool useROSCoord = false)
        {
            var npcSize = npc.Bounds.extents * 2;
            if (useROSCoord)
            {
                var temp = npcSize.x;
                npcSize.x = npcSize.z;
                npcSize.z = npcSize.y;
                npcSize.y = temp;
            }
            return new aw_monitor.msg.VehicleSize()
            {
                Name = npc.ScriptName,
                Center = ToROSVector3(npc.Bounds.center, useROSCoord),
                Size = ToROSVector3(npcSize)
            };
        }

        public static Vector3Object Vector3ToVector3Object(Vector3 input, bool useROSCoord = false)
        {
            if (useROSCoord)
                input = ROS2Utility.UnityToRosPosition(input);
            return new Vector3Object(input);
        }
        
        public static geometry_msgs.msg.Vector3 ToROSVector3(Vector3 input, bool useROSCoord = false)
        {
            if (useROSCoord)
                input = ROS2Utility.UnityToRosPosition(input);
            return new geometry_msgs.msg.Vector3()
            {
                X = input.x, Y = input.y, Z = input.z
            };
        }
    }
}