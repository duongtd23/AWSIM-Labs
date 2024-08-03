using System;

namespace AWSIM.AWAnalysis.Error
{
    public class CustomSimException : Exception
    {
        public CustomSimException()
        {
        }
        public CustomSimException(string message)
            : base(message)
        {
        }
    }
}