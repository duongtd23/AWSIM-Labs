using System;
using AWSIM_Script.Error;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;
using AWSIM.Loader;
using AWSIM.TrafficSimulation;

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
        
        private GameObject _trafficLanesParent;
        private Camera _sensorCamera;
        private TraceWriter _traceWriter;
        private bool _activated;
        private Simulation _simulation;
        
        public void Start()
        {
            _simulation = ParseSimulationScenario();
            if (_simulation != null)
            {
                InitializeAWSIM(_simulation);
            }
        }

        public void FixedUpdate()
        {
            if (!_activated && Ready())
            {
                _activated = true;
                Activate();
            }
            else if (_traceWriter != null)
            {
                CustomNPCSpawningManager.Manager()?.UpdateNPCs();
                _traceWriter?.Update();
                EgoSingletonInstance.CustomEgoSetting?.UpdateEgo();
            }
        }
        
        private bool Ready()
        {
            return EgoSingletonInstance.AutowareEgoCarGameObject != null;
        }

        private void Activate()
        {
            _trafficLanesParent = GameObject.Find("TrafficLanes");
            var lanes = Array.Empty<TrafficLane>();
            if (_trafficLanesParent != null)
                lanes = _trafficLanesParent.GetComponentsInChildren<TrafficLane>();
            CustomNPCSpawningManager.Initialize(this.gameObject,
                lanes,
                EgoSingletonInstance.AutowareEgoCarGameObject, npcTaxi, npcHatchback,
                npcSmallCar, npcTruck, npcVan,
                casualPedestrian,elegantPedestrian,
                vehicleLayerMask, groundLayerMask);
            
            _sensorCamera = EgoSingletonInstance.GetObjectDetectionCamera();

            if (_simulation != null)
            {
                PreProcessingSimulation(ref _simulation);
                ExecuteSimulation(_simulation);
                InitializeTrace(_simulation.SavingTimeout);
            }
        }
        
        private Simulation ParseSimulationScenario()
        {
            bool argDefined = CommandLineArgsManager.GetScriptArg(out string scriptFilePath);
            if (!argDefined)
            {   
                Debug.LogError("[AWAnalysis] Input script is not given. " +
                                "Specify it by argument `-script <path-to-script-file>`.");
                return null;
            }
            Debug.Log("Loading input script " + scriptFilePath);
            Simulation simulation = new ScriptParser().ParseScriptFromFile(scriptFilePath);
            return simulation;
        }

        // load AWSIM: map, ego position, etc.
        private void InitializeAWSIM(Simulation simulation)
        {
            var loader = FindObjectOfType<Loader.Loader>();
            if (loader != null)
            {
                var egoCOnfig = new EgoConfiguration()
                {
                    egoVehicleName = simulation.Ego.ModelName,
                    egoPosition = new Vector3(81381.7f, 49918.8f, 41.6f),
                    egoEulerAngles = new Vector3(0, 0, 35),
                    // egoPosition = new Vector3(81636.8f, 50572.22f, 39.26f),
                    // egoEulerAngles = new Vector3(0, 0, -80)
                };
                var simConfig = new SimulationConfiguration()
                {
                    timeScale = 1,
                    useTraffic = false
                };
                var mapConfig = new MapConfiguration()
                {
                    mapName = simulation.MapName,
                    useShadows = false
                };

                AWSIMConfiguration config = new AWSIMConfiguration()
                {
                    egoConfiguration = egoCOnfig,
                    simulationConfiguration = simConfig,
                    mapConfiguration = mapConfig
                };

                loader.CustomEgoSettings = simulation.Ego;
                loader.Activate(config);
            }
        }

        private void InitializeTrace(float savingTimeout)
        {
            bool argDefined = CommandLineArgsManager.GetTraceSavingPathArg(out string outputFilePath);
            if (!argDefined)
                Debug.LogError("[AWAnalysis] Path to save trace output is not given. " +
                               "Specify it by argument `-output <path-to-save-trace-file>`.");
            else
            {
                PerceptionMode perceptionMode = CommandLineArgsManager.GetPerceptionModeArg();
                if (ConfigLoader.Config().TraceFormat == TraceFormat.YAML)
                    _traceWriter = new YamlTraceWriter(outputFilePath,
                        _sensorCamera,
                        perceptionMode,
                        new TraceCaptureConfig(CaptureStartingTime.AW_AUTO_MODE_READY, savingTimeout));
                else
                    _traceWriter = new MaudeTraceWriter(outputFilePath,
                        _sensorCamera,
                        perceptionMode,
                        new TraceCaptureConfig(CaptureStartingTime.AW_AUTO_MODE_READY, savingTimeout));
                _traceWriter.Start();
            }
        }
        
        private void ExecuteSimulation(Simulation simulation)
        {
            foreach (NPCCar npcCar in simulation.NPCs)
            {
                CustomNPCSpawningManager.SpawnNPC(npcCar);
            }
            
            foreach (var npcPedes in simulation.Pedestrians)
            {
                CustomNPCSpawningManager.SpawnPedestrian(npcPedes);
            }

            if (simulation.Ego != null)
            {
                var customEgoSetting = new CustomEgoSetting(simulation.Ego);
                EgoSingletonInstance.SetCustomEgoSetting(customEgoSetting);
                if (FindObjectOfType<Loader.Loader>() == null)
                {
                    customEgoSetting.SetInitPose();
                }
                customEgoSetting.SetGoal();
            }
        }

        private void PreProcessingSimulation(ref Simulation simulation)
        {
            for (int i = 0; i < simulation.NPCs.Count; i++)
            {
                var npc = simulation.NPCs[i];
                if (npc.HasConfig() &&
                    npc.Config.HasALaneChange())
                {
                    if (npc.Config.LaneChange is CutInLaneChange)
                    {
                        PreProcessingCutIn(ref npc, ref simulation);
                    }
                }
            }
        }

        // compute initial position
        private void PreProcessingCutIn(ref NPCCar npc, ref Simulation simulation)
        {
            EgoDetailObject egoDetailObject = EgoSingletonInstance.GetFixedEgoDetailInfo();
            NPCDetailObject npcDetailObject = CustomNPCSpawningManager.GetNPCCarInfo(npc.VehicleType);
            var cutInLaneChange = npc.Config.LaneChange as CutInLaneChange;
            float desiredDX = cutInLaneChange.Dx;
            float timeNPCTravelBeforeCutin = ConfigLoader.Config().TimeNPCTravelBeforeCutin;
            float desiredSpeed = npc.Config.GetDesiredSpeed(cutInLaneChange.SourceLane);
            float acceleration = npc.Config.Acceleration;
            float egoSpeed = simulation.Ego.MaxVelocity;
            float npcTotalTravelDisBeforeCutin = 0.5f * desiredSpeed * desiredSpeed / acceleration +
                                                 timeNPCTravelBeforeCutin * desiredSpeed;
            float d0 = (desiredSpeed / acceleration + timeNPCTravelBeforeCutin) * egoSpeed -
                       npcTotalTravelDisBeforeCutin +
                       desiredDX +
                       (float)(egoDetailObject.extents.z + egoDetailObject.center.z) +
                       (float)(npcDetailObject.extents.z - npcDetailObject.center.z);
            // d0 is the distance from ego to NPC such that
            // if NPC starts moving when the distance between ego and NPC reaches this value,
            // the desired Dx will be satisfied
            Debug.Log($"Computed D0: {d0}");

            float egoAcceleration = ConfigLoader.Config().EgoNormalAcceleration;
            float distanceForEgoReachDesiredSpeed = 0.5f * egoSpeed * egoSpeed / egoAcceleration;
            float timeEgoTravelConstSpeed = ConfigLoader.Config().TimeEgoTravelConstSpeed;
            float distanceEgoTravelConstSpeed = timeEgoTravelConstSpeed * egoSpeed;
            Vector3 egoInitPosition = CustomSimUtils.CalculatePosition(simulation.Ego.InitialPosition);
            TrafficLane sourceLane = CustomSimUtils.ParseLane(cutInLaneChange.SourceLane);
            float egoOffsetProjectedOnSourceLane = CustomSimUtils.LongitudeDistance(
                sourceLane.Waypoints[0], 
                sourceLane.Waypoints[1] - sourceLane.Waypoints[0],
                egoInitPosition);
            
            if (Vector3.Dot(sourceLane.Waypoints[1] - sourceLane.Waypoints[0], 
                    egoInitPosition - sourceLane.Waypoints[0]) < 0)
                egoOffsetProjectedOnSourceLane = -egoOffsetProjectedOnSourceLane;

            float initialPosOffset = distanceForEgoReachDesiredSpeed +
                                     distanceEgoTravelConstSpeed +
                                     d0 +
                                     egoOffsetProjectedOnSourceLane;
            
            Debug.Log($"NPC initial position offset: {initialPosOffset}");
            
            if (sourceLane.TotalLength() <= initialPosOffset)
            {
                // TODO: handle the case when lane length is not sufficient
                throw new InvalidScriptException($"{cutInLaneChange.SourceLane}'s length is not sufficient.");
            }

            npc.InitialPosition = new LaneOffsetPosition(cutInLaneChange.SourceLane, initialPosOffset);
            cutInLaneChange.ChangeOffset = initialPosOffset + npcTotalTravelDisBeforeCutin + 
                                           (float)(npcDetailObject.extents.z + npcDetailObject.center.z -
                                                   0.3f); // TODO: remove hard code 0.3f
            npc.SpawnDelayOption = NPCDelayDistance.DelayMove(d0);
        }
    }
}