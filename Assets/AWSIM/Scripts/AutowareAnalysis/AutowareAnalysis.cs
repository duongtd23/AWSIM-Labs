using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM_Script.Error;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using AWSIM.TrafficSimulation;
using UnityEngine;

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

    public class AutowareAnalysis : MonoBehaviour
    {
        public Camera sensorCamera;
        private TraceWriter _traceWriter;
        
        // Start is called before the first frame update
        void Start()
        {
            while (CustomNPCSpawningManager.Manager() == null) ;
            var simulation = InitializeCustomSim();
            InitializeTrace(simulation.SavingTimeout);
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            _traceWriter?.Update();
            EgoSingletonInstance.GetCustomEgoSetting()?.UpdateEgo();
        }

        private Simulation InitializeCustomSim()
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
            PreProcessingSimulation(ref simulation);
            ExecuteSimulation(simulation);
            return simulation;
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
                        sensorCamera,
                        perceptionMode,
                        new TraceCaptureConfig(CaptureStartingTime.AW_AUTO_MODE_READY, savingTimeout));
                else
                    _traceWriter = new MaudeTraceWriter(outputFilePath,
                        sensorCamera,
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
                EgoSingletonInstance.SetCustomEgoSetting(new CustomEgoSetting(simulation.Ego));
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