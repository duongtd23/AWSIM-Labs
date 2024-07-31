using System.Collections;
using System.Collections.Generic;
using AWSIM.TrafficSimulation;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim
{
    public enum RelativePositionSide
    {
        LEFT,
        RIGHT
    }
    // a relative position with respect to another IPosition
    // optionally in front or behind $offset m
    public class RelativePosition : IPosition
    {
        private IPosition root;
        private RelativePositionSide side;
        private float longitudinalOffset;

        public RelativePosition(IPosition root, RelativePositionSide side, float longitudinalOffset = 0)
        {
            this.root = root;
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
            while (CustomNPCSpawningManager.Manager() == null) ;
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
            string rootLane = root.GetLane();
            float rootOffset = root.GetOffset();

            if (side == RelativePositionSide.LEFT)
                CustomSimUtils.LeftLaneOffset(rootLane, rootOffset, allLanes, out lane, out offset2);
            else
                CustomSimUtils.RightLaneOffset(rootLane, rootOffset, allLanes, out lane, out offset2);
            offset2 += this.longitudinalOffset;
            return false;
        }
    }
}