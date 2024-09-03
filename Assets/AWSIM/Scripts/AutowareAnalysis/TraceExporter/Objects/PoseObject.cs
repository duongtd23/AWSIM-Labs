using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class PoseObject
    {
        public Vector3Object position;
        public QuaternionObject quaternion;

        public bool Equals(PoseObject other)
        {
            return position.Equals(other.position) && quaternion.Equals(other.quaternion);
        }
    }
}