using System.Collections.Generic;

namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class TraceObject
    {
        public int fixedTimestep;
        public List<StateObject> states;
        public int camera_screen_width;
        public int camera_screen_height;
    }
}