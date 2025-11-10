using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim.Objects
{
    [System.Serializable]
    public class PoseWithCovarianceObject
    {
        public PoseObject pose;
        public double[] covariance;

        public PoseWithCovarianceObject(PoseObject pose, double[] covariance)
        {
            this.pose = pose;
            this.covariance = covariance;
        }
        
        public static PoseWithCovarianceObject FromRosPoseWithCovariance(geometry_msgs.msg.PoseWithCovariance input)
        {
            var pose = PoseObject.FromRosPose(input.Pose);
            var covariance = input.Covariance;
            return new PoseWithCovarianceObject(pose, covariance);
        }
    }
}