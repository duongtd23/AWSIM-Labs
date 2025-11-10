using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.DynamicCommand
{
    [System.Serializable]
    public class DynamicSpawnCommand
    {
        public string name;
        public string body_style;
        public Vector3Object position;
        public QuaternionObject orientation;

        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}