using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.DynamicCommand
{
    [System.Serializable]
    public class DynamicRemoveCommand
    {
        // if target leave unspecified, despawn all NPCs existing in the simulation
        public string target;

        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}