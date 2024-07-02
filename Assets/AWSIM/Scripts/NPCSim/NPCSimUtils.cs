using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public static class NPCSimUtils
    {
        /// <summary>
        /// return the ahead position of <param name="transform"> by a <param name="distance">
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="distance"></param>
        /// <returns></returns>
        public static Vector3 AheadPosition(Transform transform, float distance)
        {
            return transform.position + transform.forward * distance;
        }
    }
}