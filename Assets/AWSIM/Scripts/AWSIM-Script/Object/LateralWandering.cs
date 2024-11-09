using UnityEngine;

namespace AWSIM_Script.Object
{
    public class LateralWandering
    {
        public const float DUMMY_DX = -1.0f;
        // lane and offset from which the vehicle starts wandering
        public string SourceLane { get; set; }
        public float WanderOffset { get; set; }
        public Side WanderDirection { get; set; }
        public float LateralVelocity { get; set; }
        public float Velocity { get; set; }
        
        // dy: how far it laterally wanders from the lane boundary 
        public float LatitudeExceeded { get; set; }
        
        // the longitudinal distance it travels before going back to the source lane
        public float LongitudeDistance { get; set; }
        
        // the initial longitudinal distance between two vehicles
        public float Dx { get; set; } = DUMMY_DX;

        #region inner computation
        public int SourceLaneWaypointIndex { get; set; }
        
        // the waypoints that NPC will pass during the wandering.
        // It should consist of 2 waypoint
        // public Vector3[] WanderWaypoints { get; set; }
        
        // lane after going back to the source lane
        // it might be different from the source lane (maybe the next lane of the source)
        public string ReturningLane { get; set; }
        public int ReturningLaneWaypointIndex { get; set; }
        #endregion
    }
}