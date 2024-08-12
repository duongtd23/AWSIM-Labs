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
        public IPosition InitialPosition { get; set; }
        public IPosition Goal { get; set; }
    }
}
