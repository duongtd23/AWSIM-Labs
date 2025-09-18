using System;

namespace AWSIM.AWAnalysis
{
    [Serializable]
    public class AWAnalysisConfig
    {
        public AWSIMConfig AWSIMConfig { get; set; } = new ();
        public float NpcAcceleration { get; set; } = 1.5f;
        public float NpcDeceleration { get; set; } = 1.5f;
        // only write ground truth bounding box of NPC if its distance to Ego less than this config value
        public float MaxDistanceVisibleonCamera { get; set; } = 300f;
        // the interval to send engage command from the time when AW autonomous mode becomes available 
        public float DelaySendingEngageCmd { get; set; } = 1.5f;
        public float NPCPedestrianSpeed { get; set; } = 1.42f;
        public string MaudeTraceImportFile { get; set; } = "";
        public bool MaudeTraceWriteStateData { get; set; }
        public float TimeNPCTravelBeforeCutin { get; set; } = 1;
        public float EgoDefaultVelocity { get; set; } = 15 / (float)3.6;
        public float EgoNormalAcceleration { get; set; } = 1;
        public float TimeEgoTravelConstSpeed { get; set; } = 2;
        public float TimeHeadWay { get; set; } = 2;
    }
    
    [Serializable]
    public class AWSIMConfig
    {
        public string[] AllMapNames { get; set; } = { "Shinjuku", "ShinjukuNight" };

        public string[] AllEgoModelNames { get; set; } =
            { "Lexus RX450h 2015 Sample Sensor", "Lexus RX450h 2015 Sample Sensor Blue" };
    }
}