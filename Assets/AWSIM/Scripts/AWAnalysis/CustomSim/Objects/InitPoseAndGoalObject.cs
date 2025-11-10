using AWSIM.AWAnalysis.TraceExporter.Objects;
using geometry_msgs.msg;

namespace AWSIM.AWAnalysis.CustomSim.Objects
{
    [System.Serializable]
    public class InitPoseAndGoalObject
    {
        public PoseWithCovarianceObject initial_pose;
        public PoseObject goal;

        public static InitPoseAndGoalObject FromRosPoseAndGoal(PoseWithCovariance pose, Pose goal)
        {
            return new InitPoseAndGoalObject()
            {
                initial_pose = PoseWithCovarianceObject.FromRosPoseWithCovariance(pose),
                goal = PoseObject.FromRosPose(goal)
            };
        }
    }
}