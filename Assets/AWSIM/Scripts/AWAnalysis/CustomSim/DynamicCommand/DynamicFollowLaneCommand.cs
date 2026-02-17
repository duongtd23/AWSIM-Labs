using AWSIM_Script.Object;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.DynamicCommand
{
    [System.Serializable]
    public class DynamicFollowLaneCommand
    {
        public float timestamp;
        public string target;
        public string lane = ""; // if unspecified, will follow the lane currently located on
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