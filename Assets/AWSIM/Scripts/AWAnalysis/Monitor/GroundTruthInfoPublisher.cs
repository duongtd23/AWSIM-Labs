using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using ROS2;
using UnityEngine;
using aw_monitor.msg;

namespace AWSIM.AWAnalysis.Monitor
{
    public class GroundTruthInfoPublisher
    {
        protected readonly Vehicle _egoVehicle;
        protected readonly Camera _sensorCamera;
        protected readonly float _maxDistanceVisibleOnCamera;
        
        
        // inner use
        QoSSettings qosSettings = new()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 10,
        };
        string gtKinematicTopic = "/simulation/gt/kinematic";
        private GroundtruthKinematic gtKinematicMsg;
        IPublisher<GroundtruthKinematic> gtKinematicPublisher;
        
        string gtSizeTopic = "/simulation/gt/size";
        IPublisher<GroundtruthSize> gtSizePublisher;
        // cached
        private aw_monitor.msg.VehicleSize _egoSize;

        string metadataTopic = "/awsim/sim_metadata";
        IPublisher<std_msgs.msg.String> _metadataPublisher;
        private std_msgs.msg.String _metadata = new ();
        
        public GroundTruthInfoPublisher(Camera sensorCamera)
        {
            _egoVehicle = EgoSingletonInstance.AutowareEgoVehicle;
            _sensorCamera = sensorCamera;
            _maxDistanceVisibleOnCamera = ConfigLoader.Config().MaxDistanceVisibleonCamera;
            
            gtKinematicPublisher = SimulatorROS2Node.CreatePublisher<GroundtruthKinematic>(gtKinematicTopic, qosSettings.GetQoSProfile());
            gtSizePublisher = SimulatorROS2Node.CreatePublisher<GroundtruthSize>(gtSizeTopic, qosSettings.GetQoSProfile());
            _metadataPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.String>(metadataTopic, qosSettings.GetQoSProfile());

            _egoSize = StatusExtraction.GetEgoSize(EgoSingletonInstance.AutowareEgoCarGameObject, true);

            SimulatorROS2Node.CreateService<aw_monitor.srv.GroundtruthKinematic_Request, aw_monitor.srv.GroundtruthKinematic_Response>(
                "/simulation/gt_srv/kinematic",
                HandleGtKinematicRequest);
            
            SimulatorROS2Node.CreateService<aw_monitor.srv.ExecutionState_Request, aw_monitor.srv.ExecutionState_Response>(
                "/simulation/gt_srv/execution_state",
                HandleExecutionStateRequest);
        }
        
        private aw_monitor.srv.GroundtruthKinematic_Response HandleGtKinematicRequest(aw_monitor.srv.GroundtruthKinematic_Request msg)
        {
            if (gtKinematicMsg == null)
                gtKinematicMsg = new GroundtruthKinematic();
            return new aw_monitor.srv.GroundtruthKinematic_Response()
            {
                Stamp = gtKinematicMsg.Stamp,
                Groundtruth_ego = gtKinematicMsg.Groundtruth_ego,
                Groundtruth_vehicles = gtKinematicMsg.Groundtruth_vehicles,
                Groundtruth_pedestrians = gtKinematicMsg.Groundtruth_pedestrians,
            };
        }

        private aw_monitor.srv.ExecutionState_Response HandleExecutionStateRequest(
            aw_monitor.srv.ExecutionState_Request msg)
        {
            return new aw_monitor.srv.ExecutionState_Response()
            {
                Stamp = SimulatorROS2Node.GetCurrentRosTime(),
                Motion_state = ExecutionStateTracker.MotionState,
                Routing_state = ExecutionStateTracker.RoutingState,
                Operation_state = ExecutionStateTracker.OperationState,
                Is_autonomous_mode_available = ExecutionStateTracker.IsAutonomousModeAvailable,
            };
        }

        public void Publish()
        {
            PublishKinematic();
            PublishGtSize();
        }

        public void PublishKinematic()
        {
            gtKinematicMsg = ExtractKinematics();
            gtKinematicPublisher.Publish(gtKinematicMsg);
            _metadataPublisher.Publish(_metadata);
        }

        public void SetMetadataAndPublish(string data)
        {
            _metadata = new std_msgs.msg.String() { Data = data };
        }

        private GroundtruthKinematic ExtractKinematics()
        {
            var egoInfo = StatusExtraction.ExtractEgoKinematic(_egoVehicle, useROSCoord:true);
                
            var npcVehicles = CustomSimManager.GetNPCs();
            var npcInfo = StatusExtraction.ExtractNPCKinematics(npcVehicles, useROSCoord:true);
            var bboxes = StatusExtraction.Extract2DVehicleBoundingBoxes(
                npcVehicles, _sensorCamera, _egoVehicle, _maxDistanceVisibleOnCamera);
            for (int i = 0; i < npcInfo.Length; i++)
            {
                if (bboxes[i] != null)
                    npcInfo[i].bounding_box = bboxes[i];
            }

            // pedestrians
            var npcPedestrians = CustomSimManager.GetPedestrians();
            var pedesInfo = StatusExtraction.ExtractPedestrians(npcPedestrians, useROSCoord:true);
            var bboxes2 = StatusExtraction.Extract2DPedestrianBoundingBoxes(
                npcPedestrians, _sensorCamera, _egoVehicle, _maxDistanceVisibleOnCamera);
            for (int i = 0; i < pedesInfo.Length; i++)
            {
                if (bboxes2[i] != null)
                    pedesInfo[i].bounding_box = bboxes2[i];
            }

            gtKinematicMsg = new GroundtruthKinematic()
            {
                Groundtruth_ego = ToROSPoseTwistAccel(egoInfo),
                // Groundtruth_vehicles = ToROSGtInfo(npcInfo),
                // groundtruth_pedestrian = pedesInfo,
                Stamp = SimulatorROS2Node.GetCurrentRosTime()
            };
            gtKinematicMsg.Groundtruth_vehicles = new GroundtruthNPCVehicle[npcInfo.Length];
            gtKinematicMsg.Groundtruth_pedestrians = new GroundtruthNPCPedestrian[pedesInfo.Length];
            for (int i = 0; i < npcInfo.Length; i++)
            {
                gtKinematicMsg.Groundtruth_vehicles[i] = ToROSVehGtInfo(npcInfo[i]);
            }

            for (int i = 0; i < pedesInfo.Length; i++)
            {
                gtKinematicMsg.Groundtruth_pedestrians[i] = ToROSPedesGtInfo(pedesInfo[i]);
            }
            return gtKinematicMsg;
        }

        public void PublishGtSize()
        {
            // NPCs details
            var npcs = CustomSimManager.GetNPCs();
            var vehicleSizes = new aw_monitor.msg.VehicleSize[npcs.Count + 1];
            vehicleSizes[0] = _egoSize;
            for (int i = 0; i < npcs.Count; i++)
                vehicleSizes[i + 1] = StatusExtraction.GetNPCVehicleSize(npcs[i], true);

            GroundtruthSize gtSizeMsg = new GroundtruthSize()
            {
                Vehicle_sizes = vehicleSizes,
                Camera_screen_height = _sensorCamera.pixelHeight,
                Camera_screen_width = _sensorCamera.pixelWidth,
                Other_note = "",
            };
            gtSizePublisher.Publish(gtSizeMsg);
        }
        
        // public functions
        public static PoseTwistAccel ToROSPoseTwistAccel(EgoGroundTruthObject egoGroundTruthObj)
        {
            return new PoseTwistAccel()
            {
                Pose = ToROSCustomPose(egoGroundTruthObj.pose),
                Twist = ToROSTwist(egoGroundTruthObj.twist),
                Accel = ToROSAccel(egoGroundTruthObj.acceleration)
            };
        }

        public static GroundtruthNPCVehicle ToROSVehGtInfo(NPCGroundTruthObject npcGroundTruthObj)
        {
            var result = new GroundtruthNPCVehicle()
            {
                Name = npcGroundTruthObj.name,
                Pose = ToROSCustomPose(npcGroundTruthObj.pose),
                Twist = ToROSTwist(npcGroundTruthObj.twist),
                Accel = npcGroundTruthObj.acceleration,
            };
            if (npcGroundTruthObj.bounding_box != null)
                result.Bounding_box = ToROSBoundingBox(npcGroundTruthObj.bounding_box);
            return result;
        }
        
        public static GroundtruthNPCPedestrian ToROSPedesGtInfo(PedestrianGtObject pedesGtObj)
        {
            var result = new GroundtruthNPCPedestrian()
            {
                Name = pedesGtObj.name,
                Pose = ToROSCustomPose(pedesGtObj.pose),
                Speed = pedesGtObj.speed,
            };
            if (pedesGtObj.bounding_box != null)
                result.Bounding_box = ToROSBoundingBox(pedesGtObj.bounding_box);
            return result;
        }

        public static CustomPose ToROSCustomPose(Pose2Object input)
        {
            return new CustomPose()
            {
                Position = ToROSVector3(input.position),
                Rotation = ToROSVector3(input.rotation),
            };
        }

        public static geometry_msgs.msg.Twist ToROSTwist(TwistObject input)
        {
            return new geometry_msgs.msg.Twist()
            {
                Linear = ToROSVector3(input.linear),
                Angular = ToROSVector3(input.angular),
            };
        }

        public static geometry_msgs.msg.Accel ToROSAccel(AccelerationObject input)
        {
            return new geometry_msgs.msg.Accel()
            {
                Linear = ToROSVector3(input.linear),
                Angular = ToROSVector3(input.angular),
            };
        }

        public static BoundingBox ToROSBoundingBox(BoundingBoxObject input)
        {
            return new BoundingBox()
            {
                X = input.x,
                Y = input.y,
                Width = input.width,
                Height = input.height,
            };
        }

        public static geometry_msgs.msg.Vector3 ToROSVector3(Vector3Object input)
        {
            return new geometry_msgs.msg.Vector3()
            {
                X = input.x, Y = input.y, Z = input.z
            };
        }
    }
}