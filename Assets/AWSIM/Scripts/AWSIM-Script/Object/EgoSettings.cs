using System;

namespace AWSIM_Script.Object
{
	public class EgoSettings
	{
        public EgoSettings(IPosition init, IPosition goal)
        {
            InitialPosition = init;
            Goal = goal;
		}
        public EgoSettings(IPosition init, IPosition goal, float maxVelocity)
			:this(init, goal)
        {
	        MaxVelocity = maxVelocity;
        }
        public IPosition InitialPosition { get; set; }
        public IPosition Goal { get; set; }
        
        public float MaxVelocity { get; set; } // in m/s unit
    }
}
