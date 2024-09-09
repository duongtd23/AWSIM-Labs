using System;
using System.Diagnostics;
using AWSIM_Script.Object;
using System.Collections.Generic;

namespace AWSIM_Script.Object
{
	public enum VehicleType
	{
		TAXI,
		HATCHBACK,
		VAN,
		TRUCK,
		SMALL_CAR
	}
	public class NPCCar
	{
        public NPCCar(VehicleType vehicleType, IPosition spawnPosition)
        {
            VehicleType = vehicleType;
            InitialPosition = spawnPosition;
        }
        public NPCCar(VehicleType vehicleType, IPosition spawnPosition,
			IPosition goal) :
			this(vehicleType, spawnPosition)
		{
			Goal = goal;
		}
        public NPCCar(VehicleType vehicleType, IPosition spawnPosition,
			IPosition goal, string name) :
            this(vehicleType, spawnPosition, goal)
        {
			Name = name;
        }
        public NPCCar(VehicleType vehicleType, IPosition spawnPosition,
			IPosition goal, NPCConfig config) :
			this(vehicleType, spawnPosition, goal)
        {
            Config = config;
        }
        public NPCCar(VehicleType vehicleType, IPosition spawnPosition,
			IPosition goal, NPCConfig config, INPCSpawnDelay spawnDelay) :
			this(vehicleType, spawnPosition, goal, config)
        {
			SpawnDelayOption = spawnDelay;
        }
		public VehicleType VehicleType { get; set; }
        public IPosition InitialPosition { get; set; }
        public IPosition Goal { get; set; }
		public NPCConfig Config { get; set; }
        public INPCSpawnDelay SpawnDelayOption { get; set; }
		public string Name { get; set; }

		public List<Tuple<string,float>> RouteAndSpeeds => Config?.RouteAndSpeeds;

        public List<string> Route => Config?.Route;

		public bool HasGoal()
		{
			return Goal != null &&
				!Goal.Equals(LaneOffsetPosition.DummyPosition());
		}

		public bool HasConfig()
		{
			return Config != null &&
				Config.RouteAndSpeeds != null;
		}

		public bool HasDelayOption()
		{
			if (SpawnDelayOption == null)
				return false;
			if (SpawnDelayOption is NPCDelayTime npcDelayTime)
			{
				return !npcDelayTime.Equals(NPCDelayTime.DummyDelay()) &&
				       npcDelayTime.DelayType != DelayKind.NONE;
			}
			return SpawnDelayOption != null;
		}
		
		public bool NeedComputeInitialPosition()
		{
			return HasConfig() &&
			       Config.HasALaneChange() &&
			       Config.LaneChange.ChangeOffset == LaneChangeConfig.DUMMY_CHANGE_OFFSET &&
			       InitialPosition.Equals(LaneOffsetPosition.DummyPosition());
		}
    }
}

