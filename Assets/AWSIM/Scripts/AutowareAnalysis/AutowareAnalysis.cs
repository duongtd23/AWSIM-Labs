using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;
using UnityEngine.Serialization;

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
        public GameObject autowareEgoCar;
        public Camera sensorCamera;
        private TraceWriter _traceWriter;
        private CustomEgoSetting _customEgoSetting;
        
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
            _customEgoSetting.UpdateEgo();
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
                        autowareEgoCar,
                        sensorCamera,
                        perceptionMode,
                        new TraceCaptureConfig(CaptureStartingTime.AW_AUTO_MODE_READY, savingTimeout));
                else
                    _traceWriter = new MaudeTraceWriter(outputFilePath,
                        autowareEgoCar,
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

            _customEgoSetting = new CustomEgoSetting(autowareEgoCar, simulation.Ego);
        }
    }
}