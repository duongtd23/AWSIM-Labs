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
        public TraceFormat TraceFormat { get; set; } = TraceFormat.MAUDE;
        public TraceComponent[] ComponentsRecording { get; set; } = Array.Empty<TraceComponent>();
        public int PlanTrajectoryMaxStepsRecording { get; set; } = 10;
    }

    [Serializable]
    public enum TraceFormat
    {
        MAUDE = 0,
        YAML = 1
    }
    
    [Serializable]
    public enum TraceComponent
    {
        PLANNING_TRAJECTORY = 1
    }
}