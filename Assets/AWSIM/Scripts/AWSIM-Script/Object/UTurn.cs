namespace AWSIM_Script.Object
{
    public class UTurn
    {
        public const float DUMMY_DX = -1.0f;
        // lane and offset from which the vehicle starts U-Turn
        public string SourceLane { get; set; }
        public float UTurnOffset { get; set; }
        public float Velocity { get; set; }
        
        // the initial longitudinal distance between two vehicles
        public float Dx { get; set; } = DUMMY_DX;

        #region inner computation
        public int SourceLaneWaypointIndex { get; set; }
        
        public string NextLane { get; set; }
        public int NextLaneWaypointIndex { get; set; }
        #endregion
    }
}