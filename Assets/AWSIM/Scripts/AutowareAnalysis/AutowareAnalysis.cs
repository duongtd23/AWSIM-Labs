using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using AWSIM_Script.Object;
using AWSIM_Script.Parser;
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
    }

    public class AutowareAnalysis : MonoBehaviour
    {
        public GameObject autowareEgoCar;
        private TraceWriter _traceWriter;
        
        // Start is called before the first frame update
        void Start()
        {
            bool argDefined = CommandLineArgsManager.GetTraceSavingPathArg(out string outputFilePath);
            if (!argDefined)
                Debug.LogError("[AWAnalysis] Path to save trace output is not given. " +
                    "Specify it by argument `-output <path-to-save-trace-file>`.");
            else
            {
                _traceWriter = new TraceWriter(outputFilePath,
                    autowareEgoCar.GetComponent<Vehicle>());
                _traceWriter.Start();
            }
        }

        // Update is called once per frame
        void Update()
        {
            _traceWriter?.Update();
        }
    }
}