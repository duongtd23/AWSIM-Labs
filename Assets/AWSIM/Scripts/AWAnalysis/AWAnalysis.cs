using System;
using System.Linq;
using AWSIM_Script.Error;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;
using AWSIM.Loader;
using AWSIM.TrafficSimulation;
using autoware_vehicle_msgs.msg;
using autoware_adapi_v1_msgs.msg;
using AWSIM.AWAnalysis.Monitor;
using RGLUnityPlugin;

namespace AWSIM.AWAnalysis
{
    public class TopicName
    {
        public const string TOPIC_LOCALIZATION_INITIALIZATION_STATE = "/localization/initialization_state";
        public const string TOPIC_API_PERCEPTION_OBJECTS = "/api/perception/objects";
        public const string TOPIC_MISSON_PLANNING_GOAL = "/planning/mission_planning/goal";
        public const string TOPIC_PERCEPTION_RECOGNITION_OBJECTS = "/perception/object_recognition/objects";
        public const string TOPIC_API_OPERATION_MODE_STATE = "/api/operation_mode/state";
        public const string TOPIC_API_ROUTING_STATE = "/api/routing/state";
        public const string TOPIC_AUTOWARE_ENGAGE = "/autoware/engage";
        public const string TOPIC_PERCEPTION_CAMERA_OBJECTS = "/perception/object_recognition/detection/rois0";
        public const string TOPIC_INITIAL_POSE = "/initialpose";
        public const string TOPIC_MAX_VELOCITY = "/planning/scenario_planning/max_velocity";
        public const string TOPIC_PLANNING_TRAJECTORY = "/planning/scenario_planning/trajectory";
    }

    public enum PerceptionMode
    {
        LIDAR, // default
        CAMERA_LIDAR_FUSION
    }

    public class AWAnalysis : MonoBehaviour
    {
        // taxi, hatchback, small car, truck, van prefabs, respectively
        public GameObject npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan;
        public GameObject casualPedestrian, elegantPedestrian;

        [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        private LayerMask vehicleLayerMask;

        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        private LayerMask groundLayerMask;

        private Camera _sensorCamera;
        private bool _activated;

        private GroundTruthInfoPublisher _groundTruthInfoPublisher;
        public GroundTruthInfoPublisher GtInfoPublisher => _groundTruthInfoPublisher;
        private static float _timeNow;

        public void Awake()
        {
            CustomSimManager.Initialize(this.gameObject,
                npcTaxi, npcHatchback,
                npcSmallCar, npcTruck, npcVan,
                casualPedestrian, elegantPedestrian,
                vehicleLayerMask, groundLayerMask);
        }

        public void FixedUpdate()
        {
            _timeNow = Time.fixedTime;
            if (_activated)
            {
                CustomSimManager.Manager()?.UpdateNPCs();
                EgoSingletonInstance.CustomEgoSetting?.UpdateEgo();
                _groundTruthInfoPublisher?.Publish();
            }
            else if (Ready())
            {
                _activated = true;
                Activate();
                ConfigLidarNoise();
            }

            if (_groundTruthInfoPublisher == null && SimulatorROS2Node.Ok())
                InitializeSimulationPublisher();
        }

        private bool Ready()
        {
            return EgoSingletonInstance.AutowareEgoCarGameObject != null;
        }

        private void Activate()
        {
            _sensorCamera = EgoSingletonInstance.GetObjectDetectionCamera();
            CustomSimManager.InitializeEgo(EgoSingletonInstance.AutowareEgoCarGameObject);
            ExecutionStateTracker.Start();
        }

        // enable/disable noise in lidar data
        // precondition: Ego GameObject is ready (non-null)
        private void ConfigLidarNoise()
        {
            bool argDefined = CommandLineArgsManager.GetNoiseConfigArg(out bool isNoiseEnable);

            // if not defined, noise is enabled by default
            if (!argDefined)
                isNoiseEnable = true;
            Debug.Log($"[AWAnalysis] Enabling lidar noise: {isNoiseEnable}.");

            if (isNoiseEnable)
                return;

            var lidarSensors = EgoSingletonInstance.AutowareEgoCarGameObject.GetComponentsInChildren<LidarSensor>();
            foreach (var lidarSensor in lidarSensors)
            {
                lidarSensor.applyDistanceGaussianNoise = false;
                lidarSensor.applyAngularGaussianNoise = false;
            }
        }

        private void InitializeSimulationPublisher()
        {
            _groundTruthInfoPublisher = new GroundTruthInfoPublisher(_sensorCamera);
            Debug.Log("[AWAnalysis] Initialized ground truth kinematic publisher.");
        }

        public static float GetFixedTime()
        {
            return _timeNow;
        }
    }
}