using System.Collections.Generic;
using AWSIM.AWAnalysis;
using UnityEngine;

namespace AWSIM_Script.Object
{
    public class NPCPedes
    {
        public NPCPedes(string name, PedesType pedesType, List<Vector3> waypoints, float speed)
        {
            Name = name;
            PedType = pedesType;
            Waypoints = waypoints;
            Config = new NPCPedesConfig()
            {
                Speed = speed
            };
        }

        public NPCPedes(string name, PedesType pedesType, List<Vector3> waypoints)
            : this(name, pedesType, waypoints, speed: ConfigLoader.Config().NPCPedestrianSpeed)
        {}
        
        public string Name { get; set; }
        public PedesType PedType { get; set; }
        public List<Vector3> Waypoints { get; set; }
        public NPCPedesConfig Config { get; set; }
        
        // for inner computation
        public Vector3 LastPosition { get; set; }
        public Quaternion LastRotation { get; set; }
        public int CurrentWaypointIndex { get; set; } = 1;
        public Vector3 CurrentWaypoint => Waypoints[CurrentWaypointIndex];

        public bool HasDelayOption()
        {
            if (Config == null || Config.Delay == null)
                return false;
            return !Config.Delay.Equals(NPCDelayTime.DummyDelay()) &&
                   Config.Delay.DelayType != DelayKind.NONE;
        }
    }

    public enum PedesType
    {
        CASUAL,
        ELEGANT
    }
    
    public class NPCPedesConfig
    {
        public NPCDelayTime Delay { get; set; }
        public bool Loop { get; set; }
        public float Speed { get; set; }
    }
}