using System;
using System.Collections.Generic;
using AWSIM_Script.Object;

namespace AWSIM_Script.Object
{
	public class Simulation
	{
		public Simulation()
		{
			NPCs = new List<NPCCar>();
			Pedestrians = new List<NPCPedes>();
		}
		// list of NPCs
        public List<NPCCar> NPCs { get; set; }
        public List<NPCPedes> Pedestrians { get; set; }

		// Ego initial position and goal
		public EgoSettings Ego { get; set; }
		
		// indicates how long the trace will be exported when after this time the Ego does not reach goal
		// E.g., when Ego stucks at some position due to an obstacle
		public float SavingTimeout { get; set; } = DUMMY_SAVING_TIMEOUT;

		public const float DUMMY_SAVING_TIMEOUT = 0;
	}
}

