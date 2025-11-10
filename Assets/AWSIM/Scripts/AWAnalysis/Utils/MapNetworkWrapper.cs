using System;
using System.Collections.Generic;
using System.Linq;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using AWSIM.TrafficSimulation;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    [System.Serializable]
    public class MapNetworkWrapper
    {
        public string coordinate_type;
        public TrafficLaneWrapper[] traffic_lanes;

        public MapNetworkWrapper(TrafficLane[] allTrafficLanes, bool useUnityCoordinate = false)
        {
            this.coordinate_type = useUnityCoordinate ? "Unity" : "ROS";
            this.traffic_lanes = (from l in allTrafficLanes select new TrafficLaneWrapper(l, useUnityCoordinate)).ToArray();
        }
    }

    [System.Serializable]
    public class TrafficLaneWrapper
    {
        public string id;
        public Vector3Object[] waypoints;
        public TrafficLane.TurnDirectionType turn_direction;
        public float speed_limit;
        public string[] next_lanes;
        public string[] prev_lanes;
        public float width=3.5f;
        
        [System.NonSerialized]
        public bool useUnityCoordinate;
        
        public TrafficLaneWrapper(TrafficLane originLane, bool useUnityCoordinate = false)
        {
            this.id = originLane.name;
            this.turn_direction = originLane.TurnDirection;
            this.speed_limit = originLane.SpeedLimit;
            this.next_lanes = ListLaneToArray(originLane.NextLanes);
            this.prev_lanes = ListLaneToArray(originLane.PrevLanes);
            this.width = originLane.Width;
            this.useUnityCoordinate = useUnityCoordinate;
            this.waypoints = ListWaypointsToVecArray(originLane.Waypoints);
        }

        private string[] ListLaneToArray(List<TrafficLane> lanes)
        {   
            if (lanes == null)
                return Array.Empty<string>();
            return lanes.Where(lane => lane != null).Select(lane => lane.name).ToArray();
        }

        private Vector3Object[] ListWaypointsToVecArray(Vector3[] way_points)
        {
            if (way_points == null)
                return Array.Empty<Vector3Object>();
            
            if (this.useUnityCoordinate)
                return way_points.Select(point => new Vector3Object(point)).ToArray();
            return way_points.Select(point => new Vector3Object(ROS2Utility.UnityToRosMGRS(point))).ToArray();
        }
    }
}