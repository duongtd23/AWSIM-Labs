namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class BoxDetectedShapeObject : IDetectedShapeObject
    {
        public Vector3Object size;
        public readonly string shape_type = "box";
        public string DumpMaudeStr()
        {
            if (size == null)
                return "";
            return $"{size.x} {size.y} {size.z}";
        }
    }
}