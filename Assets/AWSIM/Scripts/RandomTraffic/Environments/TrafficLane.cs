using System.Collections.Generic;
using System.Text.RegularExpressions;
using AWSIM_Script.Object;
using AWSIM.AWAnalysis.CustomSim;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Traffic lane component.
    /// </summary>
    public class TrafficLane : MonoBehaviour
    {
        /// <summary>
        /// Turning direction type of vehicles.
        /// </summary>
        public enum TurnDirectionType
        {
            STRAIGHT = 0,
            LEFT = 1,
            RIGHT = 2,
            NULL = 3
        }

        [SerializeField, Tooltip("Waypoints in this lane.")]
        private Vector3[] waypoints;
        [SerializeField, Tooltip("Turning direction of vehicles in the lane.")]
        private TurnDirectionType turnDirection;
        [SerializeField, Tooltip("Next lanes connected to this lane.")]
        private List<TrafficLane> nextLanes = new List<TrafficLane>();
        [SerializeField, Tooltip("Lanes leading to this lane.")]
        private List<TrafficLane> prevLanes = new List<TrafficLane>();
        [SerializeField, Tooltip("Lanes to which vehicles in this lane should yield the right of way.")]
        private List<TrafficLane> rightOfWayLanes = new List<TrafficLane>();
        [SerializeField, Tooltip("Stop line in the lane")]
        private StopLine stopLine;
        [SerializeField, Tooltip("Speed limit in m/s")]
        private float speedLimit;
        [SerializeField, Tooltip("Is intersection lane")]
        public bool intersectionLane;
        [SerializeField, Tooltip("Lane's width. Use 3.5 as default if unset.")]
        private float width = 3.5f;
        
        public void UpdateWaypoints(Vector3[] upWaypoints)
        {
            this.waypoints = upWaypoints;
        }
        /// <summary>
        /// Get waypoints in this lane.
        /// </summary>
        public Vector3[] Waypoints => waypoints;

        /// <summary>
        /// Get turning direction of vehicles in the lane.
        /// </summary>
        public TurnDirectionType TurnDirection => turnDirection;

        /// <summary>
        /// Get next lanes connected to this lane.
        /// </summary>
        public List<TrafficLane> NextLanes => nextLanes;

        /// <summary>
        /// Get lanes leading to this lane.
        /// </summary>
        public List<TrafficLane> PrevLanes => prevLanes;

        /// <summary>
        /// Get lanes to which vehicles in this lane should yield the right of way.
        /// </summary>
        public List<TrafficLane> RightOfWayLanes => rightOfWayLanes;

        /// <summary>
        /// Get a stop line in the lane.
        /// </summary>
        public StopLine StopLine
        {
            get => stopLine;
            set => stopLine = value;
        }

        public Vector3 GetStopPoint(int waypointIndex = 0)
        {
            return StopLine == null ? Waypoints[waypointIndex] : StopLine.CenterPoint;
        }

        /// <summary>
        /// Get speed limit in m/s.
        /// </summary>
        public float SpeedLimit => speedLimit;

        /// <summary>
        /// Create <see cref="TrafficLane"/> instance in the scene.<br/>
        /// </summary>
        /// <param name="wayPoints"></param>
        /// <param name="speedLimit"></param>
        /// <returns><see cref="TrafficLane"/> instance.</returns>
        public static TrafficLane Create(Vector3[] wayPoints, TurnDirectionType turnDirection, float speedLimit = 0f)
        {
            var gameObject = new GameObject("TrafficLane", typeof(TrafficLane));
            gameObject.transform.position = wayPoints[0];
            var trafficLane = gameObject.GetComponent<TrafficLane>();
            trafficLane.waypoints = wayPoints;
            trafficLane.turnDirection = turnDirection;
            trafficLane.speedLimit = speedLimit;
            return trafficLane;
        }
        
        public float DistanceUpToWaypoint(int waypointIndex)
        {
            float distance = 0;
            for (int i = 0; i < waypointIndex; i++)
                distance += CustomSimUtils.DistanceIgnoreYAxis(waypoints[i + 1], waypoints[i]);
            return distance;
        }

        public float TotalLength()
        {
            float totalLen = 0;
            for (int i = 0; i < waypoints.Length - 1; i++)
                totalLen += CustomSimUtils.DistanceIgnoreYAxis(waypoints[i + 1], waypoints[i]);
            return totalLen;
        }

        public const float DEFAULT_WIDTH = 3.5f;
        public float Width => width == 0.0f ? DEFAULT_WIDTH : width;

        public string OriginName()
        {
            Regex r = new Regex(NPCConfig.CLONE_PATTERN);
            var matches = r.Match(name);
            if (!matches.Success || matches.Groups.Count < 2)
                return name;
            return matches.Groups[1].ToString();
        }

        public void ResetNextLanes(List<TrafficLane> _nextLanes)
        {
            this.nextLanes = _nextLanes;
        }
        
        public void ResetPrevLanes(List<TrafficLane> _prevLanes)
        {
            this.prevLanes = _prevLanes;
        }
    }
}
