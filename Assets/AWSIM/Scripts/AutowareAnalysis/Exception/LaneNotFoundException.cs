using System;

namespace AWSIM.AWAnalysis.Error
{
    public class LaneNotFoundException : Exception
    {
        public LaneNotFoundException()
        {
        }

        public LaneNotFoundException(string message)
            : base(message)
        {
        }
    }
}