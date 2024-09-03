using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class TwistObject
    {
        public Vector3Object linear;
        public Vector3Object angular;

        public bool Equals(TwistObject other)
        {
            return linear.Equals(other.linear) && angular.Equals(other.angular);
        }
    }
}