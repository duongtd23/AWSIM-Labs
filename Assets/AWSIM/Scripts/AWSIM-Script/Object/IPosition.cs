using System;
namespace AWSIM_Script.Object
{
	// abstract type for position.
	// So far, we use {lane, offset} to specify the position.
	// In future, we might add other method to specify it.
	public interface IPosition
	{
        // lane name
        public string GetLane();
        // distance from the starting point of the lane
        public float GetOffset();
    }
}

