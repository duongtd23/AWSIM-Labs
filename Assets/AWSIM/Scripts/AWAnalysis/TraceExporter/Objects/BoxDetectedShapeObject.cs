namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class BoxDetectedShapeObject : IDetectedShapeObject
    {
        public Vector3Object size;
        public string shape_type = "box";
        public string DumpMaudeStr()
        {
            if (size == null)
                return "";
            return $"{size.y} {size.z} {size.x}";
        }
    }
}