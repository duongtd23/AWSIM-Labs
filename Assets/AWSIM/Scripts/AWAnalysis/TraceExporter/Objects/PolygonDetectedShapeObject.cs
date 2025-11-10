namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class PolygonDetectedShapeObject : IDetectedShapeObject
    {
        public Vector3Object[] footprint;
        public readonly string shape_type = "polygon";
        public string DumpMaudeStr()
        {
            // if (footprint == null || footprint.Length == 0)
            return "";
        }
    }
}