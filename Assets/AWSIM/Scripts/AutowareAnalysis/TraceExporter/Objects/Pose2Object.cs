using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class Pose2Object
    {
        public Vector3Object position;
        public Vector3Object rotation;

        public bool Equals(Pose2Object other)
        {
            return position.Equals(other.position) && rotation.Equals(other.rotation);
        }
    }
}