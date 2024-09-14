namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class EgoGroundTruthObject
    {
        public Pose2Object pose;
        public TwistObject twist;
        public AccelerationObject acceleration;

        public string DumpMaudeStr()
        {
            return $"{{name: \"ego\", pose: {{{pose.DumpMaudeStr()}}}, twist: {{{twist.DumpMaudeStr()}}}, accel: {{{acceleration.DumpMaudeStr()}}}}}";
        }
    }
}