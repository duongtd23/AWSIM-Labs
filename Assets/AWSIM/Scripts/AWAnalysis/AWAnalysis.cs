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
        
        private Camera _sensorCamera;
        private TraceWriter _traceWriter;
        private bool _activated;
        private Simulation _simulation;
        private CustomEgoSetting _customEgoSetting;

        public void Awake()
        {
            CustomSimManager.Initialize(this.gameObject,
                npcTaxi, npcHatchback,
                npcSmallCar, npcTruck, npcVan,
                casualPedestrian,elegantPedestrian,
                vehicleLayerMask, groundLayerMask);
            
            _simulation = ParseSimulationScenario();
            if (_simulation != null)
            {
                InitializeAWSIM(_simulation);
                if (_simulation.Ego != null)
                {
                    _customEgoSetting = new CustomEgoSetting(_simulation.Ego);
                    EgoSingletonInstance.SetCustomEgoSetting(_customEgoSetting);
                }
            }
        }

        public void FixedUpdate()
        {
            if (!_activated && Ready())
            {
                _activated = true;
                Activate();
                InitializeEgo();
            }
            else if (_traceWriter != null)
            {
                CustomSimManager.Manager()?.UpdateNPCs();
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
            _sensorCamera = EgoSingletonInstance.GetObjectDetectionCamera();
            CustomSimManager.InitializeEgo(EgoSingletonInstance.AutowareEgoCarGameObject);

            if (_simulation != null)
            {
                PreProcessingSimulation(ref _simulation);
                ExecuteSimulation(_simulation);
                PreProcessingSwerve(ref _simulation);
                InitializeTrace(_simulation.SavingTimeout);
            }
        }

        private void InitializeEgo()
        {
            if (FindObjectOfType<Loader.Loader>() == null)
            {
                _customEgoSetting.SetInitPose();
            }
            _customEgoSetting.SetGoal();
        }
        
        public static Simulation ParseSimulationScenario()
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
                var egoConfig = new EgoConfiguration()
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
                    egoConfiguration = egoConfig,
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
                else if (ConfigLoader.Config().TraceFormat == TraceFormat.MAUDE)
                    _traceWriter = new MaudeTraceWriter(outputFilePath,
                        _sensorCamera,
                        perceptionMode,
                        new TraceCaptureConfig(CaptureStartingTime.AW_AUTO_MODE_READY, savingTimeout));
                else if (ConfigLoader.Config().TraceFormat == TraceFormat.ALL)
                    _traceWriter = new YamlAndMaudeTraceWriter(outputFilePath,
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
                CustomSimManager.SpawnNPC(npcCar);
            }
            
            foreach (var npcPedes in simulation.Pedestrians)
            {
                CustomSimManager.SpawnPedestrian(npcPedes);
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
            NPCDetailObject npcDetailObject = CustomSimManager.GetNPCCarInfo(npc.VehicleType);
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

        // mainly compute the NPCDelayDistance for $npc movement
        private void PreProcessingSwerve(ref Simulation simulation)
        {
            for (int j = 0; j < simulation.NPCs.Count; j++)
            {
                var npc = simulation.NPCs[j];
                if (npc.HasConfig() &&
                    npc.Config.LateralWandering != null &&
                    !Mathf.Approximately(npc.Config.LateralWandering.Dx, LateralWandering.DUMMY_DX) &&
                    npc.HasDelayOption() && npc.SpawnDelayOption.ActionDelayed == DelayedAction.MOVING &&
                    npc.SpawnDelayOption is NPCDelayTime delayTime &&
                    Mathf.Approximately(delayTime.DelayAmount, NPCDelayTime.DUMMY_DELAY_AMOUNT))
                {
                    DoPreProcessingSwerve(ref npc);
                }
            }
        }
        
        private void DoPreProcessingSwerve(ref NPCCar npc)
        {
            EgoDetailObject egoDetailObject = EgoSingletonInstance.GetFixedEgoDetailInfo();
            NPCDetailObject npcDetailObject = CustomSimManager.GetNPCCarInfo(npc.VehicleType);
            
            string sourceLaneStr = npc.Config.LateralWandering.SourceLane;
            var acceleration = Mathf.Approximately(npc.Config.Acceleration, NPCConfig.DUMMY_ACCELERATION)
                ? NPCVehicleConfig.Default().Acceleration
                : npc.Config.Acceleration;
            
            // distance from the spawning point to the waypoint where swerve starts
            float distance2SwerveWp = 0;
            // time required for $npc reach the waypoint where swerve starts
            // Suppose that $npc goes with constant speeds
            float time2SwerveWp = 0;
            int i = 0;
            for (; i < npc.RouteAndSpeeds.Count; i++)
            {
                string laneStr = npc.RouteAndSpeeds[i].Item1;
                if (laneStr == sourceLaneStr)
                    break;
                var lane = CustomSimUtils.ParseLane(laneStr);
                if (i == 0)
                {
                    var laneDis = lane.TotalLength() - npc.InitialPosition.GetOffset();
                    distance2SwerveWp += laneDis;
                    float speedUpTime = npc.RouteAndSpeeds[i].Item2 / acceleration;
                    float speedUpDistance = 0.5f * acceleration * speedUpTime * speedUpTime;
                    time2SwerveWp += speedUpTime + (laneDis - speedUpDistance) / npc.RouteAndSpeeds[i].Item2;
                }
                else
                {
                    var laneDis = lane.TotalLength();
                    distance2SwerveWp += laneDis;
                    time2SwerveWp += laneDis / npc.RouteAndSpeeds[i].Item2;
                }
            }

            if (i == 0)
            {
                distance2SwerveWp = npc.Config.LateralWandering.WanderOffset - 
                                    npc.InitialPosition.GetOffset() -
                                    (float)npcDetailObject.RootToFront();
                float speedUpTime = npc.RouteAndSpeeds[0].Item2 / acceleration;
                float speedUpDistance = 0.5f * acceleration * speedUpTime * speedUpTime;
                time2SwerveWp = speedUpTime;
                // this should always happen
                if (speedUpDistance < distance2SwerveWp)
                    time2SwerveWp += (distance2SwerveWp - speedUpDistance) / npc.RouteAndSpeeds[i].Item2;
            }
            else
            {
                distance2SwerveWp += npc.Config.LateralWandering.WanderOffset;
                time2SwerveWp += npc.Config.LateralWandering.WanderOffset / npc.RouteAndSpeeds[i].Item2;
            }
            Debug.Log($"[AWAnalysis] distance2SwerveWp: {distance2SwerveWp}");
            
            float distancedelay = npc.Config.LateralWandering.Dx +
                                  (float)(egoDetailObject.RootToFront() + npcDetailObject.RootToFront()) +
                                  distance2SwerveWp +
                                  (time2SwerveWp + Time.fixedDeltaTime) * EgoSingletonInstance.DesiredMaxVelocity();
            npc.SpawnDelayOption = NPCDelayDistance.DelayMove(distancedelay);
            Debug.Log($"[AWAnalysis] distance delay is: {distancedelay}");
        }
    }
}