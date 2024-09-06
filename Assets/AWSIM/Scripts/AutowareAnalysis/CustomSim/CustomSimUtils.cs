using System;
using System.Collections.Generic;
using AWSIM.AWAnalysis.Error;
using AWSIM.TrafficSimulation;
using AWSIM_Script.Object;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim
{
    public static class CustomSimUtils
    {
        public const float ANGLE_EPSILON = 5f;
        public const float MIN_LATERAL_EPSILON = 0.2f;
        public const float MAX_LATERAL_EPSILON = 7f;
        public const string TRAFFIC_LANE_STR = "trafficlane";

        /// <summary>
        /// return the distance between two points, ignore the Y component
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        public static float DistanceIgnoreYAxis(Vector3 point1, Vector3 point2)
        {
            return MagnitudeIgnoreYAxis(point1 - point2);
        }
        
        public static float MagnitudeIgnoreYAxis(Vector3 point1)
        {
            point1.y = 0f;
            return Vector3.Magnitude(point1);
        }

        // parse traffic name from a given name
        public static TrafficLane ParseLane(string laneName)
        {
            if (CustomNPCSpawningManager.Manager() != null &&
                CustomNPCSpawningManager.GetAllTrafficLanes() != null)
            {
                int laneIndex = ParseLaneIndex(laneName);
                if (laneIndex != -1 &&
                    laneIndex < CustomNPCSpawningManager.GetAllTrafficLanes().Length)
                    return CustomNPCSpawningManager.GetAllTrafficLanes()[laneIndex];
            }
            GameObject obj = GameObject.Find(laneName);
            if (obj == null)
                throw new LaneNotFoundException("[NPCSim] Cannot find traffic lane with name: " + laneName);
            return obj.GetComponent<TrafficLane>();
        }
        public static List<TrafficLane> ParseLanes(List<string> laneNames)
        {
            var lanes = new List<TrafficLane>();
            foreach (string laneName in laneNames)
                lanes.Add(ParseLane(laneName));
            return lanes;
        }

        // given "TrafficLane.231", parse to get 231
        public static int ParseLaneIndex(string laneName)
        {
            laneName = laneName.ToLower();
            if (laneName.StartsWith(TRAFFIC_LANE_STR))
            {
                laneName = laneName.Substring(TRAFFIC_LANE_STR.Length);
                if (laneName.StartsWith("."))
                    laneName = laneName.Substring(1);
                if (Int32.TryParse(laneName, out int index))
                {
                    return index;
                }
            }
            return -1;
        }

        /// <summary>
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="distance"></param>
        /// <param name="waypointIndex"></param>
        /// <returns>vector3 representing the point on lane $lane, far $distance m from the starting point of $lane</returns>
        public static Vector3 CalculatePosition(TrafficLane lane, float distance, out int waypointIndex)
        {
            float remainDistance = distance;
            for (int j = 0; j < lane.Waypoints.Length - 1; j++)
            {
                Vector3 startPoint = lane.Waypoints[j];
                Vector3 endPoint = lane.Waypoints[j + 1];
                if (DistanceIgnoreYAxis(startPoint, endPoint) < remainDistance)
                {
                    remainDistance -= DistanceIgnoreYAxis(startPoint, endPoint);
                    continue;
                }
                else
                {
                    if (remainDistance == 0)
                    {
                        waypointIndex = j;
                        return startPoint;
                    }
                    else
                    {
                        Vector3 temp = (endPoint - startPoint).normalized;
                        waypointIndex = j + 1;
                        return startPoint + (temp * remainDistance);
                    }
                }
            }
            Debug.LogWarning("The given distance " + distance + " is larger than the total lane length." +
                " The end point of the lane is used.");
            waypointIndex = lane.Waypoints.Length - 1;
            return lane.Waypoints[waypointIndex];
        }

        // if the offset of goal exceeds the lane's total length,
        // set the offset to lane length
        public static IPosition ValidateGoal(IPosition goal)
        {
            TrafficLane lane = ParseLane(goal.GetLane());
            if (goal.GetOffset() > lane.TotalLength())
            {
                return new LaneOffsetPosition(goal.GetLane(), lane.TotalLength());
            }
            return goal;
        }

        /// <summary>
        /// get the left lane of the $root lane
        /// </summary>
        /// <param name="rootLane"></param>
        /// <param name="rootOffset"></param>
        /// <param name="allLanes"></param>
        /// <param name="result"></param>
        /// <param name="offset"></param>
        /// <returns></returns>
        public static bool LeftLaneOffset(TrafficLane rootLane, float rootOffset, TrafficLane[] allLanes, out TrafficLane result, out float offset)
        {
            return SideLaneOffset(rootLane, rootOffset, allLanes,
                true, out result, out offset);
        }
        public static bool RightLaneOffset(TrafficLane rootLane, float rootOffset, TrafficLane[] allLanes, out TrafficLane result, out float offset)
        {
            return SideLaneOffset(rootLane, rootOffset, allLanes,
                false, out result, out offset);
        }

        public static bool LeftLaneOffset(string rootLane, float rootOffset, TrafficLane[] allLanes, out TrafficLane result, out float offset)
        {
            return LeftLaneOffset(ParseLane(rootLane), rootOffset, allLanes, out result, out offset);
        }
        public static bool RightLaneOffset(string rootLane, float rootOffset, TrafficLane[] allLanes, out TrafficLane result, out float offset)
        {
            return RightLaneOffset(ParseLane(rootLane), rootOffset, allLanes, out result, out offset);
        }

        /// <summary>
        /// </summary>
        /// <param name="leftSide">false indicates the right side</param>
        /// <returns></returns>
        public static bool SideLaneOffset(TrafficLane rootLane, float rootOffset, TrafficLane[] allLanes,
            bool leftSide,
            out TrafficLane result, out float offset)
        {
            if (rootLane.TurnDirection != TrafficLane.TurnDirectionType.STRAIGHT)
            {
                Debug.LogError("[NPCSim] Left lane API only supports for straight lane. " +
                    "The input lane is a " + rootLane.TurnDirection + " turn lane.");
                result = null;
                offset = 0;
                return false;
            }
            Vector3 rootPosition = CalculatePosition(rootLane, rootOffset, out int rootWaypointIndex);
            Vector2 rootDirection = DirectionIgnoreYAxis(rootLane.Waypoints[0], rootLane.Waypoints[1]);

            int laneIndex = -1;
            int waypointIndex = -1;
            float minLateralDistance = float.MaxValue;
            for (int i = 0; i < allLanes.Length; i++)
            {
                TrafficLane lane = allLanes[i];
                if (lane == rootLane) continue;
                if (lane.TurnDirection == TrafficLane.TurnDirectionType.STRAIGHT &&
                    CloseDirection(lane, rootDirection) &&
                    ProjectionOnLine(rootPosition, lane.Waypoints[0], lane.Waypoints[lane.Waypoints.Length - 1],
                        rootOffset == 0, rootPosition == rootLane.Waypoints[rootLane.Waypoints.Length - 1]))
                {
                    var lateralDistance = LateralDistance(rootPosition, lane, out waypointIndex, true);
                    if (lateralDistance < minLateralDistance)
                    {
                        if ((leftSide && OnRightSide(rootPosition,
                            lane.Waypoints[waypointIndex], lane.Waypoints[waypointIndex + 1])) ||
                            (!leftSide && OnLeftSide(rootPosition,
                            lane.Waypoints[waypointIndex], lane.Waypoints[waypointIndex + 1])))
                        {
                            laneIndex = i;
                            minLateralDistance = lateralDistance;
                        }
                    }
                }
            }
            if (!(minLateralDistance >= MIN_LATERAL_EPSILON &&
                  minLateralDistance <= MAX_LATERAL_EPSILON))
            {
                string logStr = leftSide ?
                    "left lane" : "right lane";
                Debug.LogError("Cannot find " + logStr + " of lane " + rootLane.name);
                result = null;
                offset = 0;
                return false;
            }
            result = allLanes[laneIndex];

            // compute the offset
            Vector3 waypointsLine = result.Waypoints[waypointIndex + 1] - result.Waypoints[waypointIndex];
            waypointsLine.y = 0;
            Vector3 temp = rootPosition - result.Waypoints[waypointIndex];
            temp.y = 0;
            Vector3 projection = Vector3.Project(temp, waypointsLine);
            offset = result.DistanceUpToWaypoint(waypointIndex) + projection.magnitude;
            return true;
        }

        /// <summary>
        /// check two given directions are same or close to each other
        /// </summary>
        /// <param name="direction1"></param>
        /// <param name="direction2"></param>
        /// <param name="epsilon">the error threshold allowed</param>
        /// <returns></returns>
        public static bool SameDirection(Vector2 direction1, Vector2 direction2, float epsilon = ANGLE_EPSILON)
        {
            return Vector2.Angle(direction1, direction2) < epsilon;
        }

        /// <summary>
        /// check if the lane direction is close to $direction
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="direction"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        public static bool CloseDirection(TrafficLane lane, Vector2 direction, float epsilon = ANGLE_EPSILON)
        {
            //for (int i = 0; i < lane.Waypoints.Length - 1; i++)
            //{
            //    Vector2 direction2 = DirectionIgnoreYAxis(lane.Waypoints[i], lane.Waypoints[i + 1]);
            //    if (!SameDirection(direction, direction2))
            //        return false;
            //}
            //return true;

            return SameDirection(direction,
                DirectionIgnoreYAxis(lane.Waypoints[0], lane.Waypoints[1]));
        }

        // return the direction made from $start to $end, ignoring the y components
        public static Vector2 DirectionIgnoreYAxis(Vector3 start, Vector3 end)
        {
            Vector3 temp = end - start;
            return new Vector2(temp.x, temp.z).normalized;
        }

        /// <summary>
        /// check if the projection of $position on the line made by $start and $end
        /// is between $start and $end
        /// </summary>
        /// <param name="position"></param>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="isStartingPoint">if projection of $position is close to $start</param>
        /// <param name="isEndingPoint">if projection of $position is close to end</param>
        /// <returns></returns>
        public static bool ProjectionOnLine(Vector3 position, Vector3 start, Vector3 end,
            bool isStartingPoint = false, bool isEndingPoint = false, bool ignoreYAxis = true,
            float angleEpsilon = ANGLE_EPSILON)
        {
            Vector3 line11 = position - start;
            Vector3 line12 = end - start;
            if (ignoreYAxis)
            {
                line11.y = 0;
                line12.y = 0;
            }
            float angle1 = Vector3.Angle(line11.normalized, line12.normalized);

            Vector3 line21 = position - end;
            Vector3 line22 = start - end;
            if (ignoreYAxis)
            {
                line22.y = 0;
                line21.y = 0;
            }
            float angle2 = Vector3.Angle(line21.normalized, line22.normalized);

            if (isStartingPoint)
                return angle1 < 90 + angleEpsilon && angle2 < 90 - angleEpsilon;
            else if (isEndingPoint)
                return angle1 < 90 - angleEpsilon && angle2 < 90 + angleEpsilon;
            else
                return angle1 < 90 && angle2 < 90;
        }

        /// <summary>
        /// return lateral distance from $position to the line made by $startPoint and $endPoint
        /// </summary>
        /// <param name="position"></param>
        /// <param name="startPoint"></param>
        /// <param name="endPoint"></param>
        /// <returns></returns>
        public static float LateralDistance(Vector3 position, Vector3 startPoint, Vector3 endPoint,
            bool ignoreYAxis = true)
        {
            if (ignoreYAxis)
            {
                position.y = 0;
                startPoint.y = 0;
                endPoint.y = 0;
            }
            Ray ray = new Ray(startPoint, endPoint - startPoint);
            return Vector3.Cross(ray.direction, position - ray.origin).magnitude;
        }

        // compute lateral distance from $position to $lane.
        // Pay attention to the case when lane is a curve
        public static float LateralDistance(Vector3 position, TrafficLane lane, out int waypointIndex,
            bool ignoreYAxis = true)
        {
            waypointIndex = -1;
            float minLateralDistance = float.MaxValue;
            for (int i = 0; i < lane.Waypoints.Length - 1; i++)
            {
                if (ProjectionOnLine(position, lane.Waypoints[i], lane.Waypoints[i + 1],
                        false, false, true, 3f))
                {
                    float distance = LateralDistance(position, lane.Waypoints[i], lane.Waypoints[i + 1], ignoreYAxis);
                    if (distance < minLateralDistance)
                    {
                        waypointIndex = i;
                        minLateralDistance = distance;
                    }
                }
            }
            // cannot find two waypoints of $lane such that the projection of $position is between them
            // then find the one with smallest angle
            float minAngle = float.MaxValue;
            if (waypointIndex == -1)
            {
                for (int i = 0; i < lane.Waypoints.Length - 1; i++)
                {
                    float angle = LargestAngle(position, lane.Waypoints[i], lane.Waypoints[i + 1], ignoreYAxis);
                    if (angle < minAngle)
                    {
                        waypointIndex = i;
                        minAngle = angle;
                    }
                }
            }
            return LateralDistance(position, lane.Waypoints[waypointIndex], lane.Waypoints[waypointIndex + 1], ignoreYAxis);
        }

        public static float LargestAngle(Vector3 position, Vector3 startPoint, Vector3 endPoint,
            bool ignoreYAxis = true)
        {
            if (ignoreYAxis)
            {
                position.y = 0;
                startPoint.y = 0;
                endPoint.y = 0;
            }
            Vector3 line11 = position - startPoint;
            Vector3 line12 = endPoint - startPoint;
            float angle1 = Vector3.Angle(line11.normalized, line12.normalized);

            Vector3 line21 = position - endPoint;
            Vector3 line22 = startPoint - endPoint;
            float angle2 = Vector3.Angle(line21.normalized, line22.normalized);

            return Mathf.Max(angle1, angle2);
        }

        public static bool OnLeftSide(Vector3 point, Vector3 startPoint, Vector3 endPoint)
        {
            Vector2 temp1 = new Vector2(endPoint.x - startPoint.x, endPoint.z - startPoint.z);
            Vector2 temp2 = new Vector2(point.x - startPoint.x, point.z - startPoint.z);
            return OnLeftSide(temp1, temp2);
        }

        public static bool OnRightSide(Vector3 point, Vector3 startPoint, Vector3 endPoint)
        {
            Vector2 temp1 = new Vector2(endPoint.x - startPoint.x, endPoint.z - startPoint.z);
            Vector2 temp2 = new Vector2(point.x - startPoint.x, point.z - startPoint.z);
            return OnRightSide(temp1, temp2);
        }

        // vector B is on the left side of vector A
        public static bool OnLeftSide(Vector2 A, Vector2 B)
        {
            return -A.x * B.y + A.y * B.x < 0;
        }

        // vector B is on the right side of vector A
        public static bool OnRightSide(Vector2 A, Vector2 B)
        {
            return -A.x * B.y + A.y * B.x > 0;
        }
    }
}
