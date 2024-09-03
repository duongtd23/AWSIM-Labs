namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class NPCGroundTruthObject
    {
        public string name;
        public Pose2Object pose;
        public TwistObject twist;
        public float acceleration;
        public BoundingBoxObject bounding_box; //2d bounding box on camera view
    }
}