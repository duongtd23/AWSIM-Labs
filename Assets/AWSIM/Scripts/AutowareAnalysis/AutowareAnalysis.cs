using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class AutowareAnalysis : MonoBehaviour
    {
        public GameObject autowareEgoCar;
        private GroundTruthTrace groundTruthTrace;
        private PerceptionTrace perceptionTrace;

        //private const int CAPTURE_RATE = 10; // Hz
        //private int UPDATE_INTERVAL;
        //private int stepCount = 0;

        // Start is called before the first frame update
        void Start()
        {
            //UPDATE_INTERVAL = (int)(1 / Time.fixedDeltaTime / CAPTURE_RATE);
            groundTruthTrace = new GroundTruthTrace(
                "traces/groundtrust-trace-3.maude",
                autowareEgoCar);
            groundTruthTrace.Start();

            perceptionTrace = new PerceptionTrace(
                "traces/perception-trace-3.maude");
            perceptionTrace.Start();
        }

        // Update is called once per frame
        void Update()
        {
            // stepCount = (stepCount + 1) % UPDATE_INTERVAL;
            // if (stepCount % UPDATE_INTERVAL != 1)
            //     return;
            groundTruthTrace.Update();
            perceptionTrace.Update();
        }
    }
}