using System;
using System.Collections.Generic;
using System.Linq;

namespace AWSIM_Script.Object
{
	public class NPCConfig
	{
        // this value will be replaced by the speed limit of the coressponding lane
        public const float DUMMY_SPEED = -1;
        
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
            return RouteAndSpeeds != null &&
                   RouteAndSpeeds.Exists(entry => 
                       entry.Item1 == trafficLane && entry.Item2 != NPCConfig.DUMMY_SPEED);
        }

        public bool HasALaneChange()
        {
            return LaneChange != null;
        }
        
        public bool MaintainSpeedAsEgo { get; set; }

        public float GetDesiredSpeed(string trafficLane)
        {
            if (RouteAndSpeeds == null)
                return DUMMY_SPEED;
            return RouteAndSpeeds.First(entry => entry.Item1 == trafficLane).Item2;
        }
        
        public ILaneChange LaneChange { get; set; }
        
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