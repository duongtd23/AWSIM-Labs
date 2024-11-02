using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class QuaternionObject
    {
        public double x;
        public double y;
        public double z;
        public double w;
        public QuaternionObject(double x2, double y2, double z2, double w2)
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
        
        public string DumpMaudeStr()
        {
            return $"{x} {y} {z} | {w}";
        }
    }
}