using System.Collections.Generic;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class StateObject
    {
        public double timeStamp;
        public EgoGroundTruthObject groundtruth_ego;
        public NPCGroundTruthObject[] groundtruth_NPCs;
        
        public List<PerceptionObject> perception_objects;
        public List<BBPerceptionObject> boundingbox_perception_objects;
    }
}