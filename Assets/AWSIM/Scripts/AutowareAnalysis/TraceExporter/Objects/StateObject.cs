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

        public string DumpMaudeStr()
        {
            string result = $"{timeStamp} # {{{groundtruth_ego.DumpMaudeStr()}";
            if (groundtruth_NPCs != null)   
                foreach (var npc in groundtruth_NPCs)
                    result += $", {npc.DumpMaudeStr()}";
            if (perception_objects != null)
                foreach (var perceptionObject in perception_objects)
                    result += $", {perceptionObject.DumpMaudeStr()}";
            if (boundingbox_perception_objects != null)
                foreach (var bbPerceptionObject in boundingbox_perception_objects)
                    result += $", {bbPerceptionObject.DumpMaudeStr()}";
            return result + "}";
        }
    }
}