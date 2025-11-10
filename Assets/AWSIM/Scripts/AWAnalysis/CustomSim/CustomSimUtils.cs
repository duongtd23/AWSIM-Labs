using System;
using System.Collections.Generic;
using AWSIM_Script.Error;
using AWSIM.AWAnalysis.Error;
using AWSIM.TrafficSimulation;
using AWSIM_Script.Object;
// using AWSIM.AWAnalysis.TraceExporter.Objects;
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
            if (CustomSimManager.Manager() != null &&
                CustomSimManager.GetAllTrafficLanes() != null)
            {
                int laneIndex = ParseLaneIndex(laneName);
                if (laneIndex != -1 &&
                    laneIndex < CustomSimManager.GetAllTrafficLanes().Length)
                    return CustomSimManager.GetAllTrafficLanes()[laneIndex];
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

        public static Vector3 CalculatePosition(IPosition position)
        {
            return CalculatePosition(ParseLane(position.GetLane()), position.GetOffset(), out int index);
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

        /// <summary>
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="distance"></param>
        /// <param name="waypointIndex"></param>
        /// <returns>vector3 representing the point on lane $lane, far $distance m from the starting point of $lane</returns>
        public static Vector3 CalculatePosition(List<TrafficLane> lanes, float distance,
            out int trafficLaneIndex, out int waypointIndex)
        {
            float remainDistance = distance;
            for (int i = 0; i < lanes.Count; i++)
            {
                TrafficLane lane = lanes[i];
                for (int j = 0; j < lane.Waypoints.Length - 1; j++)
                {
                    Vector3 startPoint = lane.Waypoints[j];
                    Vector3 endPoint = lane.Waypoints[j + 1];
                    if (DistanceIgnoreYAxis(startPoint, endPoint) < remainDistance)
                    {
                        remainDistance -= DistanceIgnoreYAxis(startPoint, endPoint);
                    }
                    else
                    {
                        trafficLaneIndex = i;
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
            }
            throw new LaneNotFoundException("[NPCSim] Cannot find traffic lane and offset.");
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="startPoint"></param>
        /// <param name="distance"></param>
        /// <param name="waypointIndex"> should be on $lane</param>
        /// <param name="startWaypointIndex"> the first waypoint index after $startPoint</param>
        /// <returns>the point on $lane, $distance meters far from the startPoint</returns>
        public static Vector3 CalculatePosition(TrafficLane lane, Vector3 startPoint, float distance,
            out int waypointIndex, int startWaypointIndex = 1)
        {
            float remainDistance = distance;
            for (int j = startWaypointIndex; j < lane.Waypoints.Length; j++)
            {
                Vector3 startP = j == startWaypointIndex ? startPoint : lane.Waypoints[j - 1];
                Vector3 endPoint = lane.Waypoints[j];
                if (DistanceIgnoreYAxis(startP, endPoint) < remainDistance)
                {
                    remainDistance -= DistanceIgnoreYAxis(startP, endPoint);
                }
                else
                {
                    Vector3 temp = (endPoint - startP).normalized;
                    waypointIndex = j;
                    return startP + (temp * remainDistance);
                }
            }
            waypointIndex = -1;
            return Vector3.zero;
        }

        public static float DistanceToEndingLane(TrafficLane lane, Vector3 startPoint, int startWaypointIndex = 1)
        {
            float result = 0;
            for (int id = startWaypointIndex; id < lane.Waypoints.Length; id++)
            {
                Vector3 startP = id == startWaypointIndex ? startPoint : lane.Waypoints[id - 1];
                Vector3 endPoint = lane.Waypoints[id];
                result += DistanceIgnoreYAxis(startP, endPoint);
            }
            return result;
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
        public static bool LeftLaneOffset(TrafficLane rootLane, float rootOffset, TrafficLane[] allLanes,
            out TrafficLane result, out float offset)
        {
            return SideLaneOffset(rootLane, rootOffset, allLanes,
                true, out result, out offset);
        }

        public static bool RightLaneOffset(TrafficLane rootLane, float rootOffset, TrafficLane[] allLanes,
            out TrafficLane result, out float offset)
        {
            return SideLaneOffset(rootLane, rootOffset, allLanes,
                false, out result, out offset);
        }

        public static bool LeftLaneOffset(string rootLane, float rootOffset, TrafficLane[] allLanes,
            out TrafficLane result, out float offset)
        {
            return LeftLaneOffset(ParseLane(rootLane), rootOffset, allLanes, out result, out offset);
        }

        public static bool RightLaneOffset(string rootLane, float rootOffset, TrafficLane[] allLanes,
            out TrafficLane result, out float offset)
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
                throw new InvalidScriptException("[NPCSim] Side lane API only supports for straight lane. " +
                                                 "The input lane is a " + rootLane.TurnDirection + " turn lane.");
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
                string logStr = leftSide ? "left lane" : "right lane";
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
            return LateralDistance(position, lane.Waypoints[waypointIndex], lane.Waypoints[waypointIndex + 1],
                ignoreYAxis);
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

        // calculate longitude distance from root to position according to forwardDirection vector
        public static float LongitudeDistance(Vector3 root, Vector3 forwardDirection, Vector3 position)
        {
            root.y = 0;
            position.y = 0;
            forwardDirection.y = 0;
            var angle = Vector3.Angle(forwardDirection, position - root);
            return Mathf.Cos(angle / 180 * Mathf.PI) * ((position - root).magnitude);
        }

        public static bool CalculateOffset(TrafficLane trafficLane, Vector3 position,
            out float offset, out int waypointIndex)
        {
            offset = 0;
            int i = 0;
            for (; i < trafficLane.Waypoints.Length - 1; i++)
            {
                Vector3 start = trafficLane.Waypoints[i];
                Vector3 end = trafficLane.Waypoints[i + 1];
                if (Vector3.Dot(end - start, position - end) > 0)
                {
                    offset += DistanceIgnoreYAxis(start, end);
                }
                else
                {
                    offset += DistanceIgnoreYAxis(start, position);
                    break;
                }
            }

            if (i == trafficLane.Waypoints.Length - 1)
            {
                Debug.LogError($"The position {position} exceeds the lane length.");
                waypointIndex = -1;
                offset = -1;
                return false;
            }
            waypointIndex = i + 1;
            return true;
        }

        public static bool ForwardLaneOffset(TrafficLane rootLane, Vector3 rootPosition, float distance,
            out IPosition result, out float offset, out int waypointIndex)
        {
            CalculateOffset(rootLane, rootPosition, out float offset2, out waypointIndex);
            offset = offset2 + distance;
            result = new RelativePosition(
                new LaneOffsetPosition(rootLane.name, offset2),
                RelativePositionSide.FORWARD,
                distance);
            return true;
        }

        public static float SignDistance(Vector3 root, Vector3 position, Quaternion direction)
        {
            var distance = Vector3.Distance(position, root);
            return Vector3.Dot(position - root, direction * Vector3.forward) > 0 ? distance : -distance;
        }

        // project point $point on line made by two points $start and $end
        public static Vector3 ProjectPointOnLine(Vector3 point, Vector3 start, Vector3 end)
        {
            return Vector3.Project((point - start), (end - start)) + start;
        }

        // rotate point $point around pivot $pivot an angle around Y-axis (clockwise direction)
        public static Vector3 RotatePointAroundPivot(Vector3 point, Vector3 pivot, float angle)
        {
            var v = point - pivot;
            v = Quaternion.Euler(0, angle, 0) * v;
            return pivot + v;
        }

        public static TrafficLane LaneAtPosition(Vector3 position, out int waypointId, 
            out float laneOffset, float tolerance = 0.1f)
        {
            return LaneAtPosition(position, CustomSimManager.GetAllTrafficLanes(), 
                                    out waypointId, out laneOffset, tolerance);
        }

        public static TrafficLane LaneAtPosition(Vector3 position, TrafficLane[] allTrafficLanes,
            out int waypointId, out float laneOffset, float tolerance = 0.1f)
        {
            bool checkElevation = true;
            if (Mathf.Approximately(position.z,0))
            {
                Debug.LogWarning($"[AWAnalysis] The elevation is likely not provided for point {position}." +
                                 $"The interpreted point in map might not be the expected one.");
                checkElevation = false;
            }
            List<TrafficLane> candidates = new();
            List<int> candidateWpIDs = new();
            List<float> candidateLaneOffsets = new();
            foreach (var lane in allTrafficLanes)
            {
                if (IsPointOnCenterLane(position, lane, out int wpID, out float offset, tolerance, checkElevation))
                {
                    candidates.Add(lane);
                    candidateWpIDs.Add(wpID);
                    candidateLaneOffsets.Add(offset);
                }
            }
            if (candidates.Count == 0)
            {
                waypointId = -1;
                laneOffset = -1;
                return null;
            }

            if (candidates.Count > 1)
            {
                int resultId = 0;
                float maxZ = candidates[0].Waypoints[0].z;
                string laneName = candidates[0].name;
                for (int i = 1; i < candidates.Count; i++)
                {
                    laneName += ", " + candidates[i].name;
                    if (candidates[i].Waypoints[0].z > maxZ)
                    {
                        maxZ = candidates[i].Waypoints[0].z;
                        resultId = i;
                    }
                }
                Debug.LogWarning($"[AWAnalysis] Found {candidates.Count} ({laneName}) " +
                                 $"possible traffic lanes for position {position}." +
                                 $"By default, the highest-elevation lane was selected.");
                waypointId = candidateWpIDs[resultId];
                laneOffset = candidateLaneOffsets[resultId];
                return candidates[resultId];
            }

            waypointId = candidateWpIDs[0];
            laneOffset = candidateLaneOffsets[0];
            return candidates[0];
        }

        /// <summary>
        /// check if a given point is on the lane
        /// </summary>
        /// <param name="position"></param>
        /// <param name="lane"></param>
        /// <param name="waypointId">the waypoint index of the starting point of the segment on which the point is located.</param>
        /// <param name="laneOffset">the lane offset of the point from the lane starting point</param>
        /// <param name="tolerance"></param>
        /// <returns> True if the point is on the lane center line, False otherwise.</returns>
        public static bool IsPointOnCenterLane(Vector3 position, TrafficLane lane, 
            out int waypointId, out float laneOffset, float tolerance = 0.1f, bool checkElevation = true)
        {
            Vector2 pos2D = new Vector2(position.x, position.z);
            for(int i = 0; i < lane.Waypoints.Length - 1; i++)
            {
                if (checkElevation)
                {
                    if (DistancePointToLineSegment(position, lane.Waypoints[i], lane.Waypoints[i + 1]) <= tolerance)
                    {
                        waypointId = i;
                        laneOffset = lane.DistanceUpToWaypoint(i) + Vector3.Distance(lane.Waypoints[i], position);
                        return true;
                    }
                }
                else
                {
                    Vector2 start = new Vector2(lane.Waypoints[i].x, lane.Waypoints[i].z);
                    Vector2 end = new Vector2(lane.Waypoints[i + 1].x, lane.Waypoints[i + 1].z);
                    if (DistancePointToLineSegment(pos2D, start, end) <= tolerance)
                    {
                        waypointId = i;
                        laneOffset = lane.DistanceUpToWaypoint(i) + Vector2.Distance(start, pos2D);
                        return true;
                    }
                }
            }
            waypointId = -1;
            laneOffset = -1;
            return false;
        }

        /// <summary>
        /// Calculates the shortest distance from a point to a line segment in 2D.
        /// If the projection falls outside the segment, return infinity
        /// </summary>
        /// <param name="point">The point to measure the distance from (Vector2).</param>
        /// <param name="segmentStart">The starting point of the line segment (Vector2).</param>
        /// <param name="segmentEnd">The ending point of the line segment (Vector2).</param>
        /// <returns>The shortest distance from the point to the line segment.</returns>
        public static float DistancePointToLineSegment(Vector2 point, Vector2 segmentStart, Vector2 segmentEnd)
        {
            // Vector from segmentStart to point
            Vector2 AP = point - segmentStart;

            // Vector representing the line segment
            Vector2 AB = segmentEnd - segmentStart;

            // Calculate the squared length of the segment (for normalization and avoiding sqrt early)
            float lengthSq = AB.sqrMagnitude;

            // If the segment has zero length, it's a point. Return distance to that point.
            if (lengthSq == 0.0f)
            {
                return Vector2.Distance(point, segmentStart);
            }

            // Calculate the projection parameter (t)
            // t = (AP dot AB) / |AB|^2
            float t = Vector2.Dot(AP, AB) / lengthSq;

            if (t < 0.0f || t > 1.0f)
                return float.MaxValue;

            // Calculate the closest point on the segment
            Vector2 closestPoint = segmentStart + t * AB;

            // Return the distance from the original point to the closest point on the segment
            return Vector2.Distance(point, closestPoint);
        }
        
        public static float DistancePointToLineSegment(Vector3 point, Vector3 segmentStart, Vector3 segmentEnd)
        {
            // Vector from segmentStart to point
            Vector3 AP = point - segmentStart;

            // Vector representing the line segment
            Vector3 AB = segmentEnd - segmentStart;

            // Calculate the squared length of the segment
            float lengthSq = AB.sqrMagnitude;

            // If the segment has zero length, return distance to that point
            if (lengthSq == 0.0f)
            {
                return Vector3.Distance(point, segmentStart);
            }

            // Calculate the projection parameter (t)
            float t = Vector3.Dot(AP, AB) / lengthSq;

            // If the projection falls outside the segment, return infinity
            if (t < 0.0f || t > 1.0f)
            {
                return float.MaxValue;
            }

            // Calculate the closest point on the segment
            Vector3 closestPoint = segmentStart + t * AB;

            // Return the distance from the original point to the closest point on the segment
            return Vector3.Distance(point, closestPoint);
        }
    }
}
