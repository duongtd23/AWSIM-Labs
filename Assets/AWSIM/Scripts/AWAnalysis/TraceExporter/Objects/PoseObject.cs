using System;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    [Serializable]
    public class PoseObject
    {
        public Vector3Object position;
        public QuaternionObject quaternion;

        public bool Equals(PoseObject other)
        {
            return position.Equals(other.position) && quaternion.Equals(other.quaternion);
        }

        public string DumpMaudeStr()
        {
            return $"posi: {position.DumpMaudeStr()}, qua: {quaternion.DumpMaudeStr()}";
        }

        public static PoseObject FromRosPose(geometry_msgs.msg.Pose input)
        {
            return new PoseObject()
            {
                position = new Vector3Object(input.Position.X, input.Position.Y, input.Position.Z),
                quaternion = new QuaternionObject(input.Orientation.X, input.Orientation.Y,
                    input.Orientation.Z, input.Orientation.W)
            };
        }
    }
}