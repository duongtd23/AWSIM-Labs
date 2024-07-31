using System;

namespace AWSIM.AWAnalysis.Exp
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