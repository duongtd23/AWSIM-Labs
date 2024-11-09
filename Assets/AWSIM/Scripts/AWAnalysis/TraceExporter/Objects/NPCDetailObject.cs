namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class NPCDetailObject
    {
        public string name;
        public Vector3Object center;
        public Vector3Object extents;
        
        public double RootToFront()
        {
            return center.z + extents.z;
        }
    }
}