using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    [System.Serializable]
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

        public Vector3Object(UnityEngine.Vector3 other)
        {
            this.x = other.x;
            this.y = other.y;
            this.z = other.z;
        }

        public bool Equals(Vector3Object other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);
        }

        public UnityEngine.Vector3 ToUnityVector3()
        {
            return new UnityEngine.Vector3((float)x, (float)y, (float)z);
        }

        public override string ToString()
        {
            return $"({x}, {y}, {z})";
        }

        public string DumpMaudeStr()
        {
            return $"{x} {y} {z}";
        }
    }
}