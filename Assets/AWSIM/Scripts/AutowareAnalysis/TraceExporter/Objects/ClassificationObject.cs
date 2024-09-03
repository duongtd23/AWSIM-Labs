using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class ClassificationObject
    {
        public int label;
        public float probability;

        public bool Equals(ClassificationObject other)
        {
            return label == other.label && probability.Equals(other.probability);
        }
    }
}