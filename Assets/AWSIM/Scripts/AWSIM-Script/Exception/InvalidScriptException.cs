using System;
namespace AWSIM_Script.Error
{
	public class InvalidScriptException : Exception
	{
		public InvalidScriptException()
		{
		}
        public InvalidScriptException(string message)
            : base(message)
        {
        }
    }
}

