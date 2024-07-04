using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public static class NPCSimUtils
    {
        /// <summary>
        /// return the ahead position of <param name="transform"> by a <param name="distance">
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="distance"></param>
        /// <returns></returns>
        public static Vector3 AheadPosition(Transform transform, float distance)
        {
            return transform.position + transform.forward * distance;
        }

        /// <summary>
        /// estimate the traffic lane where <param name="transform"> is located
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="lanes"></param>
        public static TrafficLane EstimateTrafficLane(Transform transform, TrafficLane[] lanes)
        {
            int re = -1;
            float smallestDistance = 1000000f;
            for (int i = 0; i < lanes.Length; i++)
            {
                for (int j = 0; j < lanes[i].Waypoints.Length - 1; j++)
                {
                    Vector3 startPoint = lanes[i].Waypoints[j];
                    Vector3 endPoint = lanes[i].Waypoints[j + 1];
                    Ray ray = new Ray(startPoint, endPoint - startPoint);

                    if (DistanceIgnoreYAxis(startPoint, transform.position) < DistanceIgnoreYAxis(startPoint, endPoint) &&
                        DistanceIgnoreYAxis(endPoint, transform.position) < DistanceIgnoreYAxis(startPoint, endPoint))
                    {

                        float distance = Vector3.Cross(ray.direction, transform.position - ray.origin).magnitude;
                        if (distance < smallestDistance)
                        {
                            re = i;
                            smallestDistance = distance;
                        }
                    }
                }
            }
            if (re == -1)
                return TrafficLane.Create(new Vector3[] { }, TrafficLane.TurnDirectionType.STRAIGHT);
            Debug.Log(smallestDistance);
            return lanes[re];
        }

        // find traffic lane by name
        public static TrafficLane FindTrafficLane(string trafficLaneName)
        {
            return GameObject.Find(trafficLaneName).GetComponent<TrafficLane>();
        }

        /// <summary>
        /// return the position and direction on lane <param name="trafficLaneName">,
        /// and far away <param name="distance"> from the start position of <param name="trafficLaneName">
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="distance"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public static Vector3 PoseOnLane(string trafficLaneName, float distance, out Quaternion direction)
        {
            TrafficLane lane = GameObject.Find(trafficLaneName).GetComponent<TrafficLane>();
            if (lane == null)
            {
                Debug.LogError("Cannot find lane with name " + trafficLaneName);
                direction = Quaternion.identity;
                return Vector3.zero;
            }
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
                    Vector3 temp = (endPoint - startPoint).normalized;
                    direction = Quaternion.LookRotation(temp);
                    return startPoint + (temp * distance);
                }
            }
            Debug.LogError("Cannot find the position far away " + distance +
                " from the starting point of lane " + trafficLaneName);
            direction = Quaternion.identity;
            return Vector3.zero;
        }

        /// <summary>
        /// Given a lane, a desired distance from the starting point of the lane,
        /// Calculate the Vector3 point
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="distance"></param>
        /// <param name="waypointIndex"></param>
        /// <returns></returns>
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
                    Vector3 temp = (endPoint - startPoint).normalized;
                    waypointIndex = j + 1;
                    return startPoint + (temp * distance);
                }
            }
            waypointIndex = -1;
            Debug.LogError("Cannot find the position far away " + distance +
                " from the starting point of lane " + lane.name);
            return Vector3.zero;
        }

        /// <summary>
        /// return the distance between two points, ignore the Y component
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        public static float DistanceIgnoreYAxis(Vector3 point1, Vector3 point2)
        {
            // in C#, objects are passed by value
            point1.y = 0f;
            point2.y = 0f;
            return Vector3.Distance(point1, point2);
        }

    }
}