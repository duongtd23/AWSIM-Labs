using System;
using System.Collections.Generic;

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

		// some other config might be added later
    }
}

