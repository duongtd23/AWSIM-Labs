using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class Vector3Object
    {
        public double x;
        public double y;
        public double z;

        public Vector3Object(double x2, double y2, double z2)
        {
            this.x = x2;
            this.y = y2;
            this.z = z2;
        }

        public bool Equals(Vector3Object other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);
        }
    }
}