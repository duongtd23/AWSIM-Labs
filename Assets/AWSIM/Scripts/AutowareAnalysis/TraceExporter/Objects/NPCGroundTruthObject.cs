namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class NPCGroundTruthObject
    {
        public string name;
        public Pose2Object pose;
        public TwistObject twist;
        public float acceleration;
        public BoundingBoxObject bounding_box; //2d bounding box on camera view

        public string DumpMaudeStr()
        {
            if (bounding_box == null)
                return $"{{name: \"{name}\", pose: {{{pose.DumpMaudeStr()}}}, twist: {{{twist.DumpMaudeStr()}}}, accel: {acceleration}}}";
            else
                return $"{{name: \"{name}\", pose: {{{pose.DumpMaudeStr()}}}, twist: {{{twist.DumpMaudeStr()}}}, accel: {acceleration}, " +
                       $"gt-rect: {bounding_box.DumpMaudeStr()}}}";
        }
    }
}