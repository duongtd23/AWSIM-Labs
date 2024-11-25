using System;

namespace AWSIM.AWAnalysis
{
    [Serializable]
    public class AWAnalysisConfig
    {
        public AWSIMConfig AWSIMConfig { get; set; }
        public float NpcAcceleration { get; set; } = 3.0f;
        public float NpcDeceleration { get; set; } = 2.0f;
        // only write ground truth bounding box of NPC if its distance to Ego less than this config value
        public float MaxDistanceVisibleonCamera { get; set; } = 130f;
        // the interval to send engage command from the time when AW autonomous mode becomes available 
        public float DelaySendingEngageCmd { get; set; } = 1.5f;
        public float NPCPedestrianSpeed { get; set; } = 1.42f;
        public TraceFormat TraceFormat { get; set; } = TraceFormat.MAUDE;
        public string MaudeTraceImportFile { get; set; }
        public bool MaudeTraceWriteStateData { get; set; }
        public float TimeNPCTravelBeforeCutin { get; set; } = 4;
        public TraceComponent[] ComponentsRecording { get; set; } = Array.Empty<TraceComponent>();
        public int PlanTrajectoryMaxStepsRecording { get; set; } = 10;
        public float EgoDefaultVelocity { get; set; } = 15 / (float)3.6;
        public float EgoNormalAcceleration { get; set; } = 1;
        public float TimeEgoTravelConstSpeed { get; set; } = 2;
        public float TimeHeadWay { get; set; } = 2;
    }

    [Serializable]
    public enum TraceFormat
    {
        YAML = 1,
        MAUDE = 2,
        ALL = 0
    }
    
    [Serializable]
    public enum TraceComponent
    {
        PLANNING_TRAJECTORY = 1,
        PREDICTION_PATHS = 2,
    }

    [Serializable]
    public class AWSIMConfig
    {
        public string[] AllMapNames { get; set; }
        public string[] AllEgoModelNames { get; set; }
    }
}