using AWSIM_Script.Object;
using AWSIM.TrafficSimulation;
using ROS2;
using UnityEngine;
using autoware_adapi_v1_msgs.msg;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class CustomEgoSetting
    {
        private GameObject _autowareEgoCar;
        private Vehicle _egoVehicle;
        public EgoSettings EgoSettings { get; }

        // for ego settings
        private Publisher<tier4_planning_msgs.msg.VelocityLimit> _maxVelPublisher;
        private tier4_planning_msgs.msg.VelocityLimit _maxVelMsg;

        public CustomEgoSetting(EgoSettings ego)
        {
            EgoSettings = ego;
            _autowareEgoCar = EgoSingletonInstance.AutowareEgoCarGameObject;
            _egoVehicle = EgoSingletonInstance.AutowareEgoCarVehicle;
        
            // set initial and goal positions for the Ego
            TrafficLane spawnLane = CustomSimUtils.ParseLane(ego.InitialPosition.GetLane());
            Vector3 initPosition = CustomSimUtils.CalculatePosition(
                spawnLane, ego.InitialPosition.GetOffset(), out int waypointIndex);
            Vector3 initFwd = waypointIndex == 0 ?
                spawnLane.Waypoints[1] - spawnLane.Waypoints[0] :
                spawnLane.Waypoints[waypointIndex] - spawnLane.Waypoints[waypointIndex - 1];
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

            SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(TopicName.TOPIC_INITIAL_POSE).Publish(poseMsg);

            // goal
            TrafficLane goalLane = CustomSimUtils.ParseLane(ego.Goal.GetLane());
            Vector3 goalPosition = CustomSimUtils.CalculatePosition(
                goalLane, ego.Goal.GetOffset(), out int waypointIndex2);
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
            
            // setting max velocity
            if (ego.MaxVelocity > 0.0)
            {
                var maxVelMsg = new tier4_planning_msgs.msg.VelocityLimit();
                maxVelMsg.Max_velocity = ego.MaxVelocity;
                maxVelMsg.Use_constraints = false;
                maxVelMsg.Constraints = new tier4_planning_msgs.msg.VelocityLimitConstraints();
                maxVelMsg.Constraints.Max_jerk = 0;
                maxVelMsg.Constraints.Min_jerk = 0;
                maxVelMsg.Constraints.Min_acceleration = 0;
                maxVelMsg.Sender = "";
                _maxVelPublisher = 
                    SimulatorROS2Node.CreatePublisher<tier4_planning_msgs.msg.VelocityLimit>(
                    TopicName.TOPIC_MAX_VELOCITY);
                _maxVelPublisher.Publish(maxVelMsg);
                _maxVelMsg = maxVelMsg;
            }
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