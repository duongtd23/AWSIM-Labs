using AWSIM_Script.Object;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.DynamicCommand
{
    [System.Serializable]
    public class DynamicFollowWaypointCommand
    {
        public string target;
        public Vector3Object[] waypoints;
        public float speed = NPCConfig.DUMMY_SPEED;
        public float acceleration = NPCConfig.DUMMY_ACCELERATION;
        public float deceleration = NPCConfig.DUMMY_DECELERATION;
        public bool is_speed_defined, is_acceleration_defined, is_deceleration_defined;
        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}