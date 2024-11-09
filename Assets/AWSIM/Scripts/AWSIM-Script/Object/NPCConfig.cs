using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.Text.RegularExpressions;

namespace AWSIM_Script.Object
{
	public class NPCConfig
	{
        // this value will be replaced by the speed limit of the coressponding lane
        public const float DUMMY_SPEED = -1;
        public const string CLONE_PATTERN = @"(.*)(\(Clone\))(_\d+)?$";

        public NPCConfig()
        {
        }

        // routes and (optional) desired speed limit
        // a map from lane name to the desired speed limit
        // if the speed limit is not set by the user, it is 0
        public List<Tuple<string, float>> RouteAndSpeeds { get; set; }

        public void UpdateRouteAndSpeeds(List<string> route)
        {
            RouteAndSpeeds = new List<Tuple<string, float>>();
            route.ForEach(lane => RouteAndSpeeds.Add(new Tuple<string, float>(lane, DUMMY_SPEED)));
        }

        public List<string> Route => RouteAndSpeeds?.ConvertAll(l => l.Item1);

        public const float DUMMY_ACCELERATION = 0;
        public const float DUMMY_DECELERATION = 0;
        public float Acceleration { get; set; } = DUMMY_ACCELERATION;
        public float Deceleration { get; set; } = DUMMY_DECELERATION;
        public bool AggresiveDrive { get; set; }

        public bool HasDesiredSpeed(string trafficLane)
        {
            if (RouteAndSpeeds != null &&
                RouteAndSpeeds.Exists(entry =>
                    trafficLane == entry.Item1 && !Mathf.Approximately(entry.Item2, DUMMY_SPEED)))
                return true;
            
            // "TrafficLane.205(Clone)" and "TrafficLane.205(Clone)_0"
            Regex r = new Regex(CLONE_PATTERN);
            var matches = r.Match(trafficLane);
            if (!matches.Success || matches.Groups.Count < 2)
                return false;
            string originLaneName = matches.Groups[1].ToString();
            return RouteAndSpeeds != null &&
                   RouteAndSpeeds.Exists(entry => 
                       originLaneName == entry.Item1 && !Mathf.Approximately(entry.Item2, DUMMY_SPEED));
        }

        public bool HasALaneChange()
        {
            return LaneChange != null;
        }
        
        public bool MaintainSpeedAsEgo { get; set; }

        public float GetDesiredSpeed(string trafficLane)
        {
            Regex r = new Regex(CLONE_PATTERN);
            var matches = r.Match(trafficLane);
            if (RouteAndSpeeds == null)
                return DUMMY_SPEED;
            foreach (var entry in RouteAndSpeeds)
            {
                if (entry.Item1 == trafficLane)
                    return entry.Item2;
                if (matches.Success && matches.Groups.Count > 1 && matches.Groups[1].ToString() == entry.Item1)
                    return entry.Item2;
            }
            return DUMMY_SPEED;
        }
        
        public ILaneChange LaneChange { get; set; }
        
        public LateralWandering LateralWandering { get; set; }
        
        public static NPCConfig DummyConfigWithoutRoute()
        {
            return new NPCConfig();
        }
    }

    public enum Side
    {
        LEFT,
        RIGHT
    }
}