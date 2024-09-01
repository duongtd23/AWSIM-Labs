using System;

namespace AWSIM.AWAnalysis
{
    [Serializable]
    public class AWAnalysisConfig
    {
        public float NpcAcceleration { get; set; } = 3.0f;
        public float NpcDeceleration { get; set; } = 2.0f;
        // only write ground truth bounding box of NPC if its distance to Ego less than this config value
        public float MaxDistanceVisibleonCamera { get; set; } = 130f;
        // the interval to send engage command from the time when AW autonomous mode becomes available 
        public float DelaySendingEngageCmd { get; set; } = 1.5f;
    }
}