using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class CameraUtils
    {
        // find mesh collider of the given NPC
        public static MeshCollider GetNPCMeshCollider(NPCVehicle npc)
        {
            var meshes = npc.GetComponentsInChildren<MeshCollider>();
            foreach (var mesh in meshes)
            {
                if (mesh.name == "BodyCollider")
                    return mesh;
            }
            Debug.LogError("[AWAnalysis] Cannot find BodyCollider");
            return meshes[0];
        }

        // capture a screenshot and save the .png file to `fileName`
        public static void Screenshot(Camera cam, string fileName)
        {
            RenderTexture screenTexture = new RenderTexture(Screen.width, Screen.height, 16);
            cam.targetTexture = screenTexture;
            RenderTexture.active = screenTexture;
            cam.Render();

            Texture2D renderedTexture = new Texture2D(Screen.width, Screen.height);
            renderedTexture.ReadPixels(new Rect(0, 0, Screen.width, Screen.height), 0, 0);
            RenderTexture.active = null;

            byte[] byteArray = renderedTexture.EncodeToPNG();
            System.IO.File.WriteAllBytes(Application.dataPath + "/" + fileName, byteArray);
        }

        /// <summary>
        /// check whether or not the world `point` is visible by the `camera`
        /// </summary>
        /// <param name="camera"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static bool PointVisibleByCamera(Camera camera, Vector3 point)
        {
            Vector3 viewportPoint = camera.WorldToViewportPoint(point);
            return (viewportPoint.z > 0 &&
                viewportPoint.x > 0 && viewportPoint.y > 0 &&
                viewportPoint.x < 1 && viewportPoint.y < 1);
        }

        /// <summary>
        /// convert local bounds of NPC to 8-point corners on world view
        /// </summary>
        /// <param name="camera"></param>
        /// <param name="npc"></param>
        /// <returns></returns>
        public static Vector3[] NPCLocalBoundsToWorldCorners(NPCVehicle npc)
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

            return worldCorners;
        }

        /// <summary>
        /// check whether the `npc` is visible by the `camera`.
        /// Even part of the vehicle is visible, true is returned.
        /// NOTE THAT this is not a precise computation,
        /// we only check if at least one of 8 corners of NPC bounds is visible
        /// to avoid the computation burden
        /// </summary>
        /// <param name="camera"></param>
        /// <param name="npc"></param>
        /// <returns></returns>
        public static bool NPCVisibleByCamera(Camera camera, NPCVehicle npc)
        {
            var worldCorners = NPCLocalBoundsToWorldCorners(npc);
            for (int i = 0; i < 8; i++)
            {
                if (PointVisibleByCamera(camera, worldCorners[i]))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// if X or Y offset of the point is outside wrt the camera view,
        /// reset it to 0 or MaxWidth or MaxHeight.
        /// However, as a trick, we use -0.1 or {Max + 0.1}
        /// to recognize it as the outside point later
        /// </summary>
        /// <param name="point"></param>
        /// <param name="camera"></param>
        /// <returns></returns>
        public static Vector3 FixScreenPoint(Vector3 point, Camera camera)
        {
            if (point.x < 0)
                point.x = -0.1f;
            if (point.y < 0)
                point.y = -0.1f;
            if (point.x > camera.pixelWidth)
                point.x = camera.pixelWidth + 0.1f;
            if (point.y > camera.pixelHeight)
                point.y = camera.pixelHeight + 0.1f;
            return point;
        }

        public static bool InRange(float value, float min, float max)
        {
            return value <= max && min <= value;
        }
    }
}