using System;
using System.Collections.Generic;
using System.IO;
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
        
        private float totalSimTime = 0;
        private float totalRealTime = 0;
        
        private float m_deltaTime = 0f;
        private float m_fps = 0f;
        private int m_updateRate = 3;
        private int m_frameCount = 0;
        private List<float> fpsList = new List<float>();
        private bool fileWritten;
        
        void Update() 
        {
            // FPS calculation
            if (ExecutionStateTracker.State < ExecutionState.AUTO_MODE_READY)
                return;
            if (ExecutionStateTracker.State == ExecutionState.GOAL_ARRIVED && !fileWritten)
            {
                fileWritten = true;
                SaveFpsLogToFile();
            }
            
            m_deltaTime += Time.unscaledDeltaTime;
            m_frameCount++;
            if (m_deltaTime > 1f / m_updateRate)
            {
                m_fps = m_frameCount / m_deltaTime;
                fpsList.Add(m_fps);
                
                // Reset variables
                m_deltaTime = 0f;
                m_frameCount = 0;
            }
        }

        private void Start()
        {
            bool ok = CommandLineArgsManager.GetNoNPCArg(out int maxNoNpc);
            if (ok && maxNoNpc > 0)
            {
                TrafficManager tm = FindObjectOfType<TrafficManager>();
                tm.AddRandomTrafficInstance(maxNoNpc);
                Debug.Log($"Enabled random traffic manager, maximum NPC: {maxNoNpc}");
            }
        }

        private void SaveFpsLogToFile()
        {
            bool ok = CommandLineArgsManager.GetLogFileArg(out string logPath);
            if (!ok || logPath == "")
                return;
            
            // Convert the list of floats to an array of strings.
            string[] lines = new string[fpsList.Count];
            for (int i = 0; i < fpsList.Count; i++)
            {
                lines[i] = fpsList[i].ToString();
            }

            // Write the lines into the file.
            File.WriteAllLines(logPath, lines);
            File.WriteAllLines("releases/file3.txt", lines);
            Debug.Log("FPS log saved to: " + logPath);
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