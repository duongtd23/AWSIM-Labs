using System;
using System.Collections.Generic;
using System.Linq;

namespace AWSIM_Script.Object
{
	public class NPCConfig
	{
        // this value will be replaced by the speed limit of the coressponding lane
        public const float DUMMY_SPEED = -1;
        
        private NPCConfig(){}
        public NPCConfig(List<string> route)
        {
            RouteAndSpeeds =new List<Tuple<string, float>>();
            route.ForEach(lane => RouteAndSpeeds.Add(new Tuple<string, float>(lane, DUMMY_SPEED)));
        }

        // routes and (optional) desired speed limit
        // a map from lane name to the desired speed limit
        // if the speed limit is not set by the user, it is 0
        public List<Tuple<string, float>> RouteAndSpeeds { get; set; }

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

        public float GetDesiredSpeed(string trafficLane)
        {
            if (RouteAndSpeeds == null)
                return DUMMY_SPEED;
            return RouteAndSpeeds.First(entry => entry.Item1 == trafficLane).Item2;
        }
        
        public LaneChangeConfig LaneChange { get; set; }
        
        public static NPCConfig DummyConfigWithoutRoute()
        {
            return new NPCConfig();
        }
    }

    public class LaneChangeConfig
    {
        public const float DEFAULT_LATERAL_VELOCITY = 1;
        public const float DUMMY_DX = -1;
        public const float DUMMY_CHANGE_OFFSET = -1;

        // the maximum is 3
        public float LateralVelocity { get; set; } = DEFAULT_LATERAL_VELOCITY;
        public float LongitudinalVelocity { get; set; }
        public string SourceLane { get; set; }
        public string TargetLane { get; set; }
        public float ChangeOffset { get; set; } = DUMMY_CHANGE_OFFSET;
        public float Dx { get; set; } = DUMMY_DX;

        public Side ChangeDirection { get; set; }
        public int SourceLaneWaypointIndex { get; set; }
        public int TargetLaneWaypointIndex { get; set; }
    }

    public enum Side
    {
        LEFT,
        RIGHT
    }
}