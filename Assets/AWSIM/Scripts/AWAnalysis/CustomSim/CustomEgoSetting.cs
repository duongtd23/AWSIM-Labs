using AWSIM_Script.Object;
using AWSIM.TrafficSimulation;
using ROS2;
using UnityEngine;
using autoware_adapi_v1_msgs.msg;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class CustomEgoSetting
    {
        // private GameObject _autowareEgoCar;
        private Vehicle _egoVehicle;
        public EgoSettings EgoSettings { get; }
        public geometry_msgs.msg.PoseWithCovarianceStamped LastInitialPose { get; private set; }
        public geometry_msgs.msg.PoseStamped LastGoal { get; private set; }

        // for ego settings
        private Publisher<tier4_planning_msgs.msg.VelocityLimit> _maxVelPublisher;
        // private Publisher<autoware_internal_planning_msgs.msg.VelocityLimit> _maxVelPublisher;
        private tier4_planning_msgs.msg.VelocityLimit _maxVelMsg;
        // private autoware_internal_planning_msgs.msg.VelocityLimit _maxVelMsg;
        
        IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> _initialPosePublisher;
        IPublisher<geometry_msgs.msg.PoseStamped> _goalPublisher;
        
        public CustomEgoSetting(EgoSettings ego)
        {
            EgoSettings = ego;
            _initialPosePublisher =
                SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(TopicName
                    .TOPIC_INITIAL_POSE);
            _goalPublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(TopicName.TOPIC_MISSON_PLANNING_GOAL);
            
            LastInitialPose = ConstructPoseMsg();
            LastGoal = ConstructGoalMsg();
            
            // setting max velocity
            if (EgoSettings.MaxVelocity > 0.0)                                                                                                                                              
            {
                var maxVelMsg = new tier4_planning_msgs.msg.VelocityLimit();
                // var maxVelMsg = new autoware_internal_planning_msgs.msg.VelocityLimit();
                maxVelMsg.Max_velocity = EgoSettings.MaxVelocity;
                maxVelMsg.Use_constraints = false;
                maxVelMsg.Constraints = new tier4_planning_msgs.msg.VelocityLimitConstraints();
                // maxVelMsg.Constraints = new autoware_internal_planning_msgs.msg.VelocityLimitConstraints();
                maxVelMsg.Constraints.Max_jerk = 0;
                maxVelMsg.Constraints.Min_jerk = 0;
                maxVelMsg.Constraints.Min_acceleration = 0;
                maxVelMsg.Sender = "";
                
                // @duongtd: 2025/05/22 Autoware Foundation migrated the topic type
                _maxVelPublisher = 
                    SimulatorROS2Node.CreatePublisher<tier4_planning_msgs.msg.VelocityLimit>(
                        TopicName.TOPIC_MAX_VELOCITY);
                // _maxVelPublisher = 
                //     SimulatorROS2Node.CreatePublisher<autoware_internal_planning_msgs.msg.VelocityLimit>(
                //         TopicName.TOPIC_MAX_VELOCITY);

                _maxVelPublisher.Publish(maxVelMsg);
                _maxVelMsg = maxVelMsg;
            }
        }

        public geometry_msgs.msg.PoseWithCovarianceStamped ConstructPoseMsg()
        {
            TrafficLane spawnLane = CustomSimUtils.ParseLane(EgoSettings.InitialPosition.GetLane());
            Vector3 initPosition = CustomSimUtils.CalculatePosition(
                spawnLane, EgoSettings.InitialPosition.GetOffset(), out int waypointIndex);
            Vector3 initFwd = waypointIndex == 0
                ? spawnLane.Waypoints[1] - spawnLane.Waypoints[0]
                : spawnLane.Waypoints[waypointIndex] - spawnLane.Waypoints[waypointIndex - 1];
            Quaternion poseRotation = Quaternion.LookRotation(initFwd);
            // Debug.Log($"Unity Position: {initPosition}, Euler angles: {poseRotation.eulerAngles}");
            
            var rosPosition = ROS2Utility.UnityToRosMGRS(initPosition);
            var rosOrientation = ROS2Utility.UnityToRosRotation(poseRotation);
            // Debug.Log($"Ros Position: {rosPosition}, Euler angles: {rosOrientation.eulerAngles}");

            var poseMsg = new geometry_msgs.msg.PoseWithCovarianceStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = "map",
                }
            };
            poseMsg.Pose = new geometry_msgs.msg.PoseWithCovariance();
            poseMsg.Pose.Pose.Position.X = rosPosition.x;
            poseMsg.Pose.Pose.Position.Y = rosPosition.y;
            poseMsg.Pose.Pose.Position.Z = rosPosition.z;
            poseMsg.Pose.Pose.Orientation.X = rosOrientation.x;
            poseMsg.Pose.Pose.Orientation.Y = rosOrientation.y;
            poseMsg.Pose.Pose.Orientation.Z = rosOrientation.z;
            poseMsg.Pose.Pose.Orientation.W = rosOrientation.w;

            poseMsg.Pose.Covariance[0] = 0.01;
            poseMsg.Pose.Covariance[7] = 0.01;
            poseMsg.Pose.Covariance[35] = 0.01;
            return poseMsg;
        }
        
        public geometry_msgs.msg.PoseStamped ConstructGoalMsg()
        {
            _egoVehicle = EgoSingletonInstance.AutowareEgoVehicle;
            TrafficLane goalLane = CustomSimUtils.ParseLane(EgoSettings.Goal.GetLane());
            Vector3 goalPosition = CustomSimUtils.CalculatePosition(
                goalLane, EgoSettings.Goal.GetOffset(), out int waypointIndex2);
            Vector3 goalFwd = waypointIndex2 == 0
                ? goalLane.Waypoints[1] - goalLane.Waypoints[0]
                : goalLane.Waypoints[waypointIndex2] - goalLane.Waypoints[waypointIndex2 - 1];
            Quaternion goalRotation = Quaternion.LookRotation(goalFwd);

            var rosGoalPos = ROS2Utility.UnityToRosMGRS(goalPosition);
            var rosOrientation = ROS2Utility.UnityToRosRotation(goalRotation);

            var goalMsg = new geometry_msgs.msg.PoseStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = "map",
                }
            };
            goalMsg.Pose = new geometry_msgs.msg.Pose();
            goalMsg.Pose.Position.X = rosGoalPos.x;
            goalMsg.Pose.Position.Y = rosGoalPos.y;
            goalMsg.Pose.Position.Z = rosGoalPos.z;
            goalMsg.Pose.Orientation.X = rosOrientation.x;
            goalMsg.Pose.Orientation.Y = rosOrientation.y;
            goalMsg.Pose.Orientation.Z = rosOrientation.z;
            goalMsg.Pose.Orientation.W = rosOrientation.w;
            return goalMsg;
        }

        public void SetInitPose(geometry_msgs.msg.PoseWithCovarianceStamped poseMsg)
        {
            // _autowareEgoCar = EgoSingletonInstance.AutowareEgoCarGameObject;
            // _egoVehicle = EgoSingletonInstance.AutowareEgoVehicle;

            // set initial pose
            // var poseMsgHeader = poseMsg as MessageWithHeader;
            // SimulatorROS2Node.UpdateROSTimestamp(ref poseMsgHeader);
            _initialPosePublisher.Publish(poseMsg);
            
            // _egoVehicle.SetPosition(ROS2Utility.RosMGRSToUnityPosition(poseMsg.Pose.Pose.Position));
            // _egoVehicle.SetRotation(ROS2Utility.RosToUnityRotation(poseMsg.Pose.Pose.Orientation));
        }
        
        // public void SetGoal(geometry_msgs.msg.PoseStamped goalMsg)
        // {
        //     _goalPublisher.Publish(goalMsg);
        // }

        public void UpdateEgo()
        {
            if (_maxVelPublisher != null && _maxVelMsg != null)
            {
                // Debug.Log("[AWAnalysis] Setting max velocity...");
                _maxVelPublisher.Publish(_maxVelMsg);
            }
        }
    }
}