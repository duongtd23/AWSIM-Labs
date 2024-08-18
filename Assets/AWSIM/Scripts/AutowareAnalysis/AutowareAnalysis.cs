using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
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
    }

    public class AutowareAnalysis : MonoBehaviour
    {
        public GameObject autowareEgoCar;
        private TraceWriter _traceWriter;
        
        //private const int CAPTURE_RATE = 10; // Hz
        //private int UPDATE_INTERVAL;
        //private int stepCount = 0;

        // Start is called before the first frame update
        void Start()
        {
            //UPDATE_INTERVAL = (int)(1 / Time.fixedDeltaTime / CAPTURE_RATE);
            _traceWriter = new TraceWriter(
                ConfigLoader.Config().traceFileName,
                autowareEgoCar.GetComponent<Vehicle>());
            _traceWriter.Start();
        }

        // Update is called once per frame
        void Update()
        {
            // stepCount = (stepCount + 1) % UPDATE_INTERVAL;
            // if (stepCount % UPDATE_INTERVAL != 1)
            //     return;
            _traceWriter.Update();
        }
    }
}