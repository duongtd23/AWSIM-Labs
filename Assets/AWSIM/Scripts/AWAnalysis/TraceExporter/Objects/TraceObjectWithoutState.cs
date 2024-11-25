namespace AWSIM.AWAnalysis.TraceExporter.Objects
{
    public class TraceObjectWithoutState
    {
        public int fixedTimestep;
        public int camera_screen_width;
        public int camera_screen_height;
        public EgoDetailObject ego_detail;
        public NPCDetailObject[] npcs_detail;
        public IOtherComponent other;
        public string comment;

        public TraceObjectWithoutState Clone()
        {
            return new TraceObjectWithoutState()
            {
                fixedTimestep = this.fixedTimestep,
                camera_screen_width = this.camera_screen_width,
                camera_screen_height = this.camera_screen_height,
                ego_detail = this.ego_detail,
                npcs_detail = this.npcs_detail,
                other = this.other,
                comment = this.comment
            };
        }
    }
}