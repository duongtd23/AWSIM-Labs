namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class PredictPathObject
    {
        public double confidence;
        // [sec] time step for each path step
        public double time_step;
        public Pose2Object[] path;

        public string DumpMaudeStr()
        {
            return $"confidence: {confidence}, time-step: {time_step}, path: {DumpPathStr()}";
        }

        public string DumpPathStr()
        {
            if (path == null || path.Length == 0)
                return "nil";

            string pathStr = $"{{{path[0].DumpMaudeStr()}}}";
            for (int i = 1; i < path.Length; i++)
            {
                pathStr += $" {{{path[i].DumpMaudeStr()}}}";
            }

            return pathStr;
        }
    }
}