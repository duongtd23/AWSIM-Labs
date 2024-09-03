using AWSIM.AWAnalysis.CustomSim;
using AWSIM.TrafficSimulation;

namespace AWSIM_Script.Object
{
    public enum RelativePositionSide
    {
        LEFT,
        RIGHT,
        FORWARD
    }
    // a relative position with respect to another IPosition
    // optionally in front or behind $offset m
    public class RelativePosition : IPosition
    {
        private IPosition referencePosition;
        private RelativePositionSide side;
        private float longitudinalOffset;

        public RelativePosition(IPosition referencePosition, RelativePositionSide side, float longitudinalOffset = 0)
        {
            this.referencePosition = referencePosition;
            this.side = side;
            this.longitudinalOffset = longitudinalOffset;
        }

        public string GetLane()
        {
            ToLaneOffsetPosition(out LaneOffsetPosition laneOffset);
            return laneOffset.GetLane();
        }

        public float GetOffset()
        {
            ToLaneOffsetPosition(out LaneOffsetPosition laneOffset);
            return laneOffset.GetOffset();
        }

        public bool ToLaneOffsetPosition(out LaneOffsetPosition laneOffset)
        {
            while (CustomNPCSpawningManager.Manager() == null);
            return ToLaneOffsetPosition(CustomNPCSpawningManager.GetAllTrafficLanes(), out laneOffset);
        }

        // convert to LaneOffsetPosition
        public bool ToLaneOffsetPosition(TrafficLane[] allLanes, out LaneOffsetPosition laneOffset)
        {
            ToLaneOffsetPosition(allLanes, out TrafficLane lane, out float offset2);
            laneOffset = new LaneOffsetPosition(lane.name, offset2);
            return true;
        }

        // derive the lane and the offset
        public bool ToLaneOffsetPosition(TrafficLane[] allLanes, out TrafficLane lane, out float offset2)
        {
            string rootLane = referencePosition.GetLane();
            float rootOffset = referencePosition.GetOffset();

            if (side == RelativePositionSide.LEFT)
                CustomSimUtils.LeftLaneOffset(rootLane, rootOffset, allLanes, out lane, out offset2);
            else if (side == RelativePositionSide.RIGHT)
                CustomSimUtils.RightLaneOffset(rootLane, rootOffset, allLanes, out lane, out offset2);
            else
            {
                lane = CustomSimUtils.ParseLane(rootLane);
                offset2 = rootOffset;
            }
            offset2 += this.longitudinalOffset;
            return false;
        }
    }
}