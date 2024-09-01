using System;
using System.Collections.Generic;
namespace AWSIM_Script.Object
{
	public class NPCConfig
	{
        // this value will be replaced by the speed limit of the coressponding lane
        public const float DUMMY_SPEED = -1;
        
        private NPCConfig(){}
        public NPCConfig(List<string> route)
        {
            RouteAndSpeeds = new Dictionary<string, float>();
            route.ForEach(lane => RouteAndSpeeds.Add(lane, DUMMY_SPEED));
        }
		public NPCConfig(Dictionary<string, float> routeAndSpeeds)
		{
			RouteAndSpeeds = routeAndSpeeds;
		}

        public NPCConfig(List<string> route, Dictionary<string, float> routeAndSpeeds)
            :this(routeAndSpeeds)
        {
            foreach(string lane in route)
            {
                if (!RouteAndSpeeds.ContainsKey(lane))
                    RouteAndSpeeds.Add(lane, DUMMY_SPEED);
            }
        }

        // routes and (optional) desired speed limit
        // a map from lane name to the desired speed limit
        // if the speed limit is not set by the user, it is 0
        public Dictionary<string, float> RouteAndSpeeds { get; set; }

        public List<string> Route()
        {
            if (RouteAndSpeeds == null)
                return null;
            return new List<string>(RouteAndSpeeds.Keys);
        }

        public const float DUMMY_ACCELERATION = 0;
        public const float DUMMY_DECELERATION = 0;
        public float Acceleration { get; set; } = DUMMY_ACCELERATION;
        public float Deceleration { get; set; } = DUMMY_DECELERATION;
        public bool AggresiveDrive { get; set; }

        public static NPCConfig DummyConfigWithoutRoute()
        {
            return new NPCConfig();
        }
    }
}


