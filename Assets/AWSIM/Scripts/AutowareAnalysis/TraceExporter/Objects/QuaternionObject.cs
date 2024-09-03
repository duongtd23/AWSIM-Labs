using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class QuaternionObject
    {
        public float x;
        public float y;
        public float z;
        public float w;
        public QuaternionObject(float x2, float y2, float z2, float w2)
        {
            this.x = x2;
            this.y = y2;
            this.z = z2;
            this.w = w2;
        }

        public bool Equals(QuaternionObject other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && w.Equals(other.w);
        }
    }
}