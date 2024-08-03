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
			IPosition goal, NPCConfig config, NPCSpawnDelay spawnDelay) :
			this(vehicleType, spawnPosition, goal, config)
        {
			SpawnDelayOption = spawnDelay;
        }
        public IPosition InitialPosition { get; set; }
        public IPosition Goal { get; set; }
		public NPCSpawnDelay SpawnDelayOption { get; set; }
		public NPCConfig Config { get; set; }
		public VehicleType VehicleType { get; set; }
		public string Name { get; set; }

		public Dictionary<string,float> RouteAndSpeeds()
		{
			if (Config == null)
				return null;
			return Config.RouteAndSpeeds;
		}

        public List<string> Route()
        {
            if (Config == null)
                return null;
            var routeSpeeds = Config.RouteAndSpeeds;
			if (routeSpeeds == null)
				return null;
			return new List<string>(routeSpeeds.Keys);
        }
    }
}

