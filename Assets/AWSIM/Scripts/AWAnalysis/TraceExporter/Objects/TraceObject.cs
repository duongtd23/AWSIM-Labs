using System.Collections.Generic;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class TraceObject : TraceObjectWithoutState
    {
        public List<StateObject> states;
    }
}