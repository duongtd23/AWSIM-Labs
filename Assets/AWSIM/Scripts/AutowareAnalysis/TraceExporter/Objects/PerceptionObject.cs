using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class PerceptionObject
    {
        public int[] id;
        public float existence_prob;
        public ClassificationObject[] classification;
        public Pose2Object pose;
        public TwistObject twist;
        public AccelerationObject acceleration;

        public bool Equals(PerceptionObject other)
        {
            return IDEqual(other.id) && existence_prob.Equals(other.existence_prob) && 
                   ClassEqual(other.classification) && pose.Equals(other.pose) && 
                   twist.Equals(other.twist) && acceleration.Equals(other.acceleration);
        }

        public bool IDEqual(int[] otherID)
        {
            for (int i = 0; i < id.Length; ++i)
                if (id[i] != otherID[i])
                    return false;
            return true;
        }
        
        public bool ClassEqual(ClassificationObject[] otherClass)
        {
            if (classification.Length != otherClass.Length)
                return false;
            for (int i = 0; i < classification.Length; ++i)
                if (!classification[i].Equals(otherClass[i]))
                    return false;
            return true;
        }
    }
}