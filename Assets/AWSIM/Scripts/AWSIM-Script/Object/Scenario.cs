using System;
using System.Collections.Generic;
using AWSIM_Script.Object;

namespace AWSIM_Script.Object
{
	public class Scenario
	{
		public Scenario()
		{
			NPCs = new List<NPCCar>();
        }
		// list of NPCs
        public List<NPCCar> NPCs { get; set; }

		// Ego initial position and goal
		public EgoSettings Ego { get; set; }

		// some other config might be added later
    }
}

