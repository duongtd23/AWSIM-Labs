namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class PedestrianGtObject
    {
        public string name;
        public Pose2Object pose;
        public float speed;
        public BoundingBoxObject bounding_box; //2d bounding box on camera view

        public string DumpMaudeStr()
        {
            if (bounding_box == null)
                return $"{{name: \"{name}\", pose: {{{pose.DumpMaudeStr()}}}, speed: {speed}}}";
            else
                return $"{{name: \"{name}\", pose: {{{pose.DumpMaudeStr()}}}, speed: {speed}, " +
                       $"gt-rect: {bounding_box.DumpMaudeStr()}}}";
        }
    }
}