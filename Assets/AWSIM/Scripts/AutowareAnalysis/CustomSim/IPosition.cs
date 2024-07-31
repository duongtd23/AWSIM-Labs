using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim
{
    public interface IPosition
    {
        // lane name
        public string GetLane();
        // distance from the starting point of the lane
        public float GetOffset();
    }
}