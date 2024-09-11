namespace AWSIM_Script.Object
{
    public class CutInLaneChange : ILaneChange
    {
        public float LateralVelocity { get; set; } = ILaneChange.DEFAULT_LATERAL_VELOCITY;
        public float LongitudinalVelocity { get; set; }
        public string SourceLane { get; set; }
        public string TargetLane { get; set; }
        public float ChangeOffset { get; set; } = ILaneChange.DUMMY_CHANGE_OFFSET;
        public float Dx { get; set; } = ILaneChange.DUMMY_DX;

        public Side ChangeDirection { get; set; }
        public int SourceLaneWaypointIndex { get; set; }
        public int TargetLaneWaypointIndex { get; set; }
    }
}