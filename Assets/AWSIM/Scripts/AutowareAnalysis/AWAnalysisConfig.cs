using System;

namespace AWSIM.AWAnalysis
{
    [Serializable]
    public class AWAnalysisConfig
    {
        public string traceFileName;
        public float captureLength;
        public float npcAcceleration;
        public float npcDeceleration;
    }
}