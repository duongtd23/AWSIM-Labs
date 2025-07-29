using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.DynamicCommand
{
    [System.Serializable]
    public class DynamicRemoveCommand
    {
        public string target;

        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}