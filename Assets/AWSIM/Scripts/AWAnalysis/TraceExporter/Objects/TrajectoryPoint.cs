namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class TrajectoryPoint
    {
        public double time_from_start;
        public PoseObject pose;
        public float longitudinal_velocity;
        public float lateral_velocity;
        public float acceleration;
    }
}