using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class BBPerceptionObject
    {
        public float existence_prob;
        public ClassificationObject[] classification;
        public BoundingBoxObject bounding_box;

        public bool Equals(BBPerceptionObject other)
        {
            return existence_prob.Equals(other.existence_prob) && ClassEqual(other.classification) && bounding_box.Equals(other.bounding_box);
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