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
        public EgoSettings EgoSettings { get; private set; }

        // for ego settings
        private Publisher<autoware_internal_planning_msgs.msg.VelocityLimit> _maxVelPublisher;
        private autoware_internal_planning_msgs.msg.VelocityLimit _maxVelMsg;

        public CustomEgoSetting(EgoSettings ego)
        {
            EgoSettings = ego;
            // setting max velocity
            if (EgoSettings.MaxVelocity > 0.0)
            {
                var maxVelMsg = new autoware_internal_planning_msgs.msg.VelocityLimit();
                maxVelMsg.Max_velocity = EgoSettings.MaxVelocity;
                maxVelMsg.Use_constraints = false;
                maxVelMsg.Constraints = new autoware_internal_planning_msgs.msg.VelocityLimitConstraints();
                maxVelMsg.Constraints.Max_jerk = 0;
                maxVelMsg.Constraints.Min_jerk = 0;
                maxVelMsg.Constraints.Min_acceleration = 0;
                maxVelMsg.Sender = "";
                
                // @duongtd: 2025/05/22 Autoware Foundation migrated the topic type
                // _maxVelPublisher = 
                //     SimulatorROS2Node.CreatePublisher<tier4_planning_msgs.msg.VelocityLimit>(
                //         TopicName.TOPIC_MAX_VELOCITY);
                _maxVelPublisher = 
                    SimulatorROS2Node.CreatePublisher<autoware_internal_planning_msgs.msg.VelocityLimit>(
                        TopicName.TOPIC_MAX_VELOCITY);

                _maxVelPublisher.Publish(maxVelMsg);
                _maxVelMsg = maxVelMsg;
            }
        }

        public void SetInitPose()
        {
            // _autowareEgoCar = EgoSingletonInstance.AutowareEgoCarGameObject;
            _egoVehicle = EgoSingletonInstance.AutowareEgoVehicle;

            // set initial pose
            TrafficLane spawnLane = CustomSimUtils.ParseLane(EgoSettings.InitialPosition.GetLane());
            Vector3 initPosition = CustomSimUtils.CalculatePosition(
                spawnLane, EgoSettings.InitialPosition.GetOffset(), out int waypointIndex);
            Vector3 initFwd = waypointIndex == 0
                ? spawnLane.Waypoints[1] - spawnLane.Waypoints[0]
                : spawnLane.Waypoints[waypointIndex] - spawnLane.Waypoints[waypointIndex - 1];
            Quaternion poseRotation = Quaternion.LookRotation(initFwd);

            _egoVehicle.SetPosition(initPosition);
            _egoVehicle.SetRotation(poseRotation);

            var mgrsOffset = Environment.Instance.MgrsOffsetPosition;
            var poseMsg = new geometry_msgs.msg.PoseWithCovarianceStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = "map",
                }
            };
            poseMsg.Pose = new geometry_msgs.msg.PoseWithCovariance();
            poseMsg.Pose.Pose.Position.X = initPosition.z + mgrsOffset.x;
            poseMsg.Pose.Pose.Position.Y = -initPosition.x + mgrsOffset.y;
            poseMsg.Pose.Pose.Position.Z = 0;
            poseMsg.Pose.Pose.Orientation.X = -poseRotation.z;
            poseMsg.Pose.Pose.Orientation.Y = poseRotation.x;
            poseMsg.Pose.Pose.Orientation.Z = -poseRotation.y;
            poseMsg.Pose.Pose.Orientation.W = poseRotation.w;

            var poseMsgHeader = poseMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref poseMsgHeader);

            SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(TopicName.TOPIC_INITIAL_POSE)
                .Publish(poseMsg);
        }
        
        public void SetGoal()
        {
            _egoVehicle = EgoSingletonInstance.AutowareEgoVehicle;
            var mgrsOffset = Environment.Instance.MgrsOffsetPosition;
            TrafficLane goalLane = CustomSimUtils.ParseLane(EgoSettings.Goal.GetLane());
            Vector3 goalPosition = CustomSimUtils.CalculatePosition(
                goalLane, EgoSettings.Goal.GetOffset(), out int waypointIndex2);
            Vector3 goalFwd = waypointIndex2 == 0 ?
                goalLane.Waypoints[1] - goalLane.Waypoints[0] :
                goalLane.Waypoints[waypointIndex2] - goalLane.Waypoints[waypointIndex2 - 1];
            Quaternion goalRotation = Quaternion.LookRotation(goalFwd);

            var goalMsg = new geometry_msgs.msg.PoseStamped()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = "map",
                }
            };
            goalMsg.Pose = new geometry_msgs.msg.Pose();
            goalMsg.Pose.Position.X = goalPosition.z + mgrsOffset.x;
            goalMsg.Pose.Position.Y = -goalPosition.x + mgrsOffset.y;
            goalMsg.Pose.Position.Z = 0;
            goalMsg.Pose.Orientation.X = -goalRotation.z;
            goalMsg.Pose.Orientation.Y = goalRotation.x;
            goalMsg.Pose.Orientation.Z = -goalRotation.y;
            goalMsg.Pose.Orientation.W = goalRotation.w;

            SimulatorROS2Node.CreateSubscription<LocalizationInitializationState>(
            TopicName.TOPIC_LOCALIZATION_INITIALIZATION_STATE, msg =>
            {
                if (msg.State == LocalizationInitializationState.INITIALIZED)
                {
                    // Debug.Log("[AWAnalysis] Setting goal for Ego...");
                    var goalMsgHeader = goalMsg as MessageWithHeader;
                    SimulatorROS2Node.UpdateROSTimestamp(ref goalMsgHeader);
                    SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(TopicName.TOPIC_MISSON_PLANNING_GOAL).Publish(goalMsg);
                }
            });
        }

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