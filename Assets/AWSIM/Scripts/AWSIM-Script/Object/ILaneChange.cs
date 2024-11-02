namespace AWSIM_Script.Object
{
    public interface ILaneChange
    {
        public const float DEFAULT_LATERAL_VELOCITY = 1;
        public const float DUMMY_DX = -1;
        public const float DUMMY_CHANGE_OFFSET = -1;
        
        // the maximum is 3
        public float LateralVelocity { get; set; }
        public float LongitudinalVelocity { get; set; }
        public string SourceLane { get; set; }
        public string TargetLane { get; set; }
        public float ChangeOffset { get; set; }
        
        // inner computation
        public Side ChangeDirection { get; set; }
        public int SourceLaneWaypointIndex { get; set; }
        public int TargetLaneWaypointIndex { get; set; }
    }
}