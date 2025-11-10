using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class AccelerationObject
    {
        public Vector3Object linear;
        public Vector3Object angular;

        public bool Equals(AccelerationObject other)
        {
            return linear.Equals(other.linear) && angular.Equals(other.angular);
        }
        
        public string DumpMaudeStr()
        {
            return $"lin: {linear.DumpMaudeStr()}, ang: {angular.DumpMaudeStr()}";
        }
    }
}