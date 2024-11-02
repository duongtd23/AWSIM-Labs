using System.Collections.Generic;
using UnityEngine;
using AWSIM.TrafficSimulation;
using AWSIM.AWAnalysis.Error;

namespace AWSIM.AWAnalysis
{
    public static class AutowareAnalysisUtils
    {
        // suppose stamp2 >= stamp1
        public static float DiffInMiliSec(builtin_interfaces.msg.Time stamp1, builtin_interfaces.msg.Time stamp2)
        {
            return stamp2.Sec * 1000 + stamp2.Nanosec / 1000000 -
                    stamp1.Sec * 1000 - stamp2.Nanosec / 1000000;
        }
    }
}