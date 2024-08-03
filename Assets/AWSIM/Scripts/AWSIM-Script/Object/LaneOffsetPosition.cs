using System;
namespace AWSIM_Script.Object
{
    /// <summary>
    /// A pair of lane and offset position from the starting point of the lane
    /// This is used for many purpose, e.g., to specify location for spawning NPCs
    /// </summary>
    public class LaneOffsetPosition : IPosition
    {
        private string laneName;
        private float offset;
        public LaneOffsetPosition(string laneName, float offset = 0)
        {
            this.laneName = laneName;
            this.offset = offset;
        }

        public string GetLane() => laneName;

        public float GetOffset() => offset;

        public void SetLane(string laneName)
        {
            this.laneName = laneName;
        }
        public void SetOffset(float offset)
        {
            this.offset = offset;
        }

        public static LaneOffsetPosition DummyPosition()
        {
            return new LaneOffsetPosition("", 0);
        }
        public override bool Equals(object obj)
        {
            if (obj is LaneOffsetPosition)
            {
                var obj2 = (LaneOffsetPosition)obj;
                return this.GetLane() == obj2.GetLane() &&
                    this.GetOffset() == obj2.GetOffset();
            }    
            return base.Equals(obj);
        }
        public override string ToString()
        {
            return laneName + " at " + offset;
        }
    }
}

