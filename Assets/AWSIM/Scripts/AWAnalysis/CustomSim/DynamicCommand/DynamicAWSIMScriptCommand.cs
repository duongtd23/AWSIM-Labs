using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.DynamicCommand
{
    [System.Serializable]
    public class DynamicAWSIMScriptCommand
    {
        public string file;
        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}