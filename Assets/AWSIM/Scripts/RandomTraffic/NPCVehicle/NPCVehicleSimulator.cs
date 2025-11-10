using System;
using System.Collections.Generic;
using System.Linq;
using AWSIM_Script.Error;
using AWSIM_Script.Object;
using AWSIM.AWAnalysis.CustomSim;
using UnityEngine;
using UnityEngine.Profiling;
using Object = UnityEngine.Object;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// This class simulates states of NPC vehicles and updates visualization via <see cref="NPCVehicle"/>.
    /// The process of the simulation consists of three steps:<br/>
    /// - Cognition step implemented in <see cref="NPCVehicleCognitionStep"/><br/>
    /// - Decision step implemented in <see cref="NPCVehicleDecisionStep"/><br/>
    /// - Control step implemented in <see cref="NPCVehicleControlStep"/><br/>
    /// Each step updates <see cref="NPCVehicleInternalState"/> and the result is passed to <see cref="NPCVehicleVisualizationStep"/> for visualization update.
    /// </summary>
    public class NPCVehicleSimulator : IDisposable
    {

        /// <summary>
        /// Get NPC vehicle states that are updated in simulation steps.<br/>
        /// </summary>
        public IReadOnlyList<NPCVehicleInternalState> VehicleStates
            => vehicleStates;

        /// <summary>
        /// Get or set EGO Vehicle that should be considered in the simulation.
        /// </summary>
        public Transform EGOVehicle { get; set; }
        public int maxVehicleCount;

        private List<NPCVehicleInternalState> vehicleStates;
        private NPCVehicleCognitionStep cognitionStep;
        private NPCVehicleDecisionStep decisionStep;
        private NPCVehicleControlStep controlStep;
        private NPCVehicleVisualizationStep visualizationStep;
        private Transform dummyEgo;

        public NPCVehicleSimulator(NPCVehicleConfig config,
            LayerMask vehicleLayerMask,
            LayerMask groundLayerMask,
            int maxVehicleCount,
            GameObject egoVehicle)
        {
            vehicleStates = new List<NPCVehicleInternalState>();
            cognitionStep = new NPCVehicleCognitionStep(vehicleLayerMask, groundLayerMask, maxVehicleCount);
            decisionStep = new NPCVehicleDecisionStep(config);
            controlStep = new NPCVehicleControlStep(config);
            visualizationStep = new NPCVehicleVisualizationStep();
            this.maxVehicleCount = maxVehicleCount;
            EGOVehicle = egoVehicle.transform;
        }


        /// <summary>
        /// When there is no real Ego vehicle in a scene, dummy one must be set.
        /// </summary>
        public void UnregisterEgo()
        {
            if (dummyEgo)
            {
                EGOVehicle = dummyEgo;
            }
        }

        /// <summary>
        /// Dummy ego is used when there is no real Ego in the scene
        /// </summary>
        public void SetDummyEgo(GameObject ego)
        {
            dummyEgo = ego.transform;
        }

        /// <summary>
        /// Registers Ego vehicle.
        /// </summary>
        public void RegisterEgo(GameObject egoVehicle)
        {
            EGOVehicle = egoVehicle.transform;
        }

        /// <summary>
        /// Register <see cref="NPCVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="lane">Initial lane of the vehicle</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(NPCVehicle vehicle, TrafficLane lane, int waypointIndex)
        {
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, lane, waypointIndex));
        }
        
        public void Register(NPCVehicle vehicle, TrafficLane lane, int waypointIndex, NPCConfig customConfig)
        {
            var internalState = NPCVehicleInternalState.Create(vehicle, lane, waypointIndex);
            internalState.CustomConfig = customConfig;
            vehicleStates.Add(internalState);
        }

        /// <summary>
        /// Register <see cref="NPCVehicle"/> to be updated by the simulator.
        /// </summary>
        /// <param name="vehicle">The vehicle to be registered</param>
        /// <param name="route">Route for vehicle to follow</param>
        /// <param name="waypointIndex">Current waypoint index of the vehicle</param>
        public void Register(NPCVehicle vehicle, List<TrafficLane> route, int waypointIndex)
        {
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, route, waypointIndex));
        }

        /// <summary>
        /// This should be called every time any vehicle is destroyed.
        /// </summary>
        public void RemoveInvalidVehicles()
        {
            (VehicleStates as List<NPCVehicleInternalState>)?.RemoveAll(IsVehicleNull);

            // check if vehicle is null
            bool IsVehicleNull(NPCVehicleInternalState state)
            {
                return state.Vehicle == null;
            }
        }

        /// <summary>
        /// Execute simulation steps and update visualization.
        /// </summary>
        /// <param name="deltaTime">Simulation time step</param>
        public void StepOnce(float deltaTime)
        {
            // Simulation steps
            Profiler.BeginSample("NPCVehicleSimulator.Cognition");
            cognitionStep.Execute(vehicleStates, EGOVehicle);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Decision");
            decisionStep.Execute(VehicleStates);
            Profiler.EndSample();

            Profiler.BeginSample("NPCVehicleSimulator.Control");
            controlStep.Execute(VehicleStates, deltaTime);
            Profiler.EndSample();

            // Visualization step
            Profiler.BeginSample("NPCVehicleSimulator.Visualize");
            visualizationStep.Execute(VehicleStates, EGOVehicle);
            Profiler.EndSample();
        }

        /// <summary>
        /// Show editor gizmos for debugging.
        /// </summary>
        public void ShowGizmos(bool showYieldingPhase, bool showObstacleChecking)
        {
            decisionStep.ShowGizmos(VehicleStates);
            cognitionStep.ShowGizmos(VehicleStates, showYieldingPhase, showObstacleChecking);
        }

        public void ClearAll()
        {
            foreach (var state in VehicleStates)
            {
                state.ShouldDespawn = true;
            }
        }

        public void Dispose()
        {
            cognitionStep?.Dispose();
        }
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="waypointIndex"></param>
        /// <param name="goal">must be validated (offet does not exceed total length)</param>
        /// <param name="customConfig">its Route and RouteAndSpeeds must be non-null</param>
        public void Register(NPCVehicle vehicle, int waypointIndex,
            IPosition goal, NPCConfig customConfig, VehicleType vehicleType = VehicleType.HATCHBACK)
        {
            var routeStr = customConfig.Route;
            var route = CustomSimUtils.ParseLanes(routeStr);
            
            // if there is a lane change
            if (customConfig.HasALaneChange())
            {
                TrafficLane sourceLane = route.Find(l => l.name == customConfig.LaneChange.SourceLane);
                float changeOffset = customConfig.LaneChange.ChangeOffset;
                
                // add a waypoint where lane change starts
                int sourceWaypointId = AddaWaypointToSourceLane(ref sourceLane, changeOffset);

                var laneChange = customConfig.LaneChange;
                // add a waypoint to the point where lane change complete
                int targetWaypointId = AddaWaypointToTargetLaneChange(ref route, ref laneChange);
                customConfig.LaneChange = laneChange;
                
                customConfig.LaneChange.SourceLaneWaypointIndex = sourceWaypointId;
                customConfig.LaneChange.TargetLaneWaypointIndex = targetWaypointId;
            }

            // Swerve behavior
            // Clone the source lane and modify its waypoints
            else if (customConfig.LateralWandering != null)
            {
                // TrafficLane sourceLane = route.Find(l => l.name == customConfig.LateralWandering.SourceLane);
                var sourceLaneIndex = route.FindIndex(l => l.name == customConfig.LateralWandering.SourceLane);
                var sourceLaneClone = Object.Instantiate(route[sourceLaneIndex]);
                
                float wanderOffset = customConfig.LateralWandering.WanderOffset;
                
                // add a waypoint where swerve starts
                int sourceWaypointId = AddaWaypointToSourceLane(ref sourceLaneClone, wanderOffset);
                customConfig.LateralWandering.SourceLaneWaypointIndex = sourceWaypointId;

                var updatedLateralWandering = customConfig.LateralWandering;
                TrafficLane nextLaneClone = null;
                int nextLaneIndex = 0;
                for (; nextLaneIndex < route.Count; nextLaneIndex++)
                    if (route[nextLaneIndex].name == customConfig.LateralWandering.SourceLane)
                        break;
                nextLaneIndex++;
                if (nextLaneIndex < route.Count)
                {
                    nextLaneClone = Object.Instantiate(route[nextLaneIndex]);
                }
                
                ComputeWanderingWaypoints(ref sourceLaneClone, sourceWaypointId, 
                    ref updatedLateralWandering, ref nextLaneClone,
                    vehicle);
                customConfig.LateralWandering = updatedLateralWandering;
                route[sourceLaneIndex] = sourceLaneClone;
                if (nextLaneClone != null && nextLaneIndex < route.Count)
                    route[nextLaneIndex] = nextLaneClone;
            }
            
            // U-Turn behavior
            // Clone the source lane and modify its waypoints
            else if (customConfig.UTurn != null)
            {
                var sourceLaneIndex = route.FindIndex(l => l.name == customConfig.UTurn.SourceLane);
                var sourceLaneClone = Object.Instantiate(route[sourceLaneIndex]);
                
                float uTurnOffset = customConfig.UTurn.UTurnOffset;
                
                // add a waypoint where U-Turn starts
                int sourceWaypointId = AddaWaypointToSourceLane(ref sourceLaneClone, uTurnOffset);
                customConfig.UTurn.SourceLaneWaypointIndex = sourceWaypointId;

                var nextLane = route[sourceLaneIndex + 1];
                customConfig.UTurn.NextLane = nextLane.name;

                // compute turning radius, and update waypoints for the next lane
                Tuple<float, float> wheelBaseAndTurningAngle =
                    CustomSimManager.GetWheelBaseAndTurningWheelAngle(vehicleType);
                var rightUTurn = CustomSimUtils.OnLeftSide(sourceLaneClone.Waypoints[sourceWaypointId],
                    nextLane.Waypoints[1], nextLane.Waypoints[0]);
                
                // modify waypoints of the source lane to make the turn
                int nextWaypointId = AddUTurnWaypoints(ref sourceLaneClone, sourceWaypointId,
                    ref nextLane,
                    wheelBaseAndTurningAngle.Item1, wheelBaseAndTurningAngle.Item2, rightUTurn);

                customConfig.UTurn.NextLaneWaypointIndex = nextWaypointId;
                route[sourceLaneIndex] = sourceLaneClone;
            }
            vehicleStates.Add(NPCVehicleInternalState.Create(vehicle, route, goal, 
                customConfig, waypointIndex));
        }

        private int AddaWaypointToSourceLane(ref TrafficLane sourceLane, float offset)
        {
            Vector3 newWaypoint = CustomSimUtils.CalculatePosition(sourceLane, offset, out int waypointIndex);
            Debug.Log($"[AWAnalysis] Adding waypoint {newWaypoint} at {waypointIndex} to source lane.");
            var updateWaypoints = new List<Vector3>(sourceLane.Waypoints);
            updateWaypoints.Insert(waypointIndex,newWaypoint);
            sourceLane.UpdateWaypoints(updateWaypoints.ToArray());
            return waypointIndex;
        }
        
        private int AddaWaypointToTargetLaneChange(ref List<TrafficLane> route, ref ILaneChange laneChangeConfig)
        {
            TrafficLane sourceLane = route[0];
            int firstTargetLaneIndex = 0;
            for (int i = 0; i < route.Count; i++)
            {
                var lane = route[i];
                if (lane.name == laneChangeConfig.SourceLane)
                    sourceLane = lane;
                if (lane.name == laneChangeConfig.TargetLane)
                    firstTargetLaneIndex = i;
            }

            var ok = CustomSimUtils.SideLaneOffset(sourceLane, laneChangeConfig.ChangeOffset,
                new TrafficLane[1] { route[firstTargetLaneIndex] },
                laneChangeConfig.ChangeDirection == Side.LEFT,
                out TrafficLane other, out float offset);
            if (!ok)
            {
                throw new InvalidScriptException("Cannot parse the lane change information");
            }
            
            float timeForLaneChange = sourceLane.Width / laneChangeConfig.LateralVelocity;
            float longitudeLaneChangeDistance = timeForLaneChange * laneChangeConfig.LongitudinalVelocity;
            
            Debug.Log($"[AWAnalysis] Longitude distance for Lane Change: {longitudeLaneChangeDistance}");

            // sequence of lanes after lane-change
            var targetLaneSequence = route.GetRange(firstTargetLaneIndex, route.Count - firstTargetLaneIndex);
            Vector3 newWaypoint = CustomSimUtils.CalculatePosition(targetLaneSequence, 
                offset + longitudeLaneChangeDistance, 
                out int targetLaneIndex,
                out int waypointIndex);
            
            // remove redundant lanes
            if (targetLaneIndex > 0)
                route.RemoveRange(firstTargetLaneIndex, targetLaneIndex);

            var updateWaypoints = new List<Vector3>(route[firstTargetLaneIndex].Waypoints);
            updateWaypoints.Insert(waypointIndex, newWaypoint);
            route[firstTargetLaneIndex].UpdateWaypoints(updateWaypoints.ToArray());
            laneChangeConfig.TargetLane = route[firstTargetLaneIndex].name;
            return waypointIndex;
        }

        // compute the waypoints on the wandering (2 points) and
        // determine the lane after going back along with the waypoint index
        private void ComputeWanderingWaypoints(ref TrafficLane sourceLane, int sourceWaypointId, 
            ref LateralWandering lateralWandering, ref TrafficLane nextLane,
            NPCVehicle vehicle)
        {
            Vector3 vehDirection = sourceWaypointId == 0 ?
                sourceLane.Waypoints[1] - sourceLane.Waypoints[0] :
                sourceLane.Waypoints[sourceWaypointId + 1] - sourceLane.Waypoints[sourceWaypointId];
            var originRot = Quaternion.LookRotation(vehDirection, Vector3.up).eulerAngles;

            var vehicleHalfWidth = vehicle.GetCarInfo().extents.x;
            var rotateRadian = (float)Math.Asin(lateralWandering.LateralVelocity / lateralWandering.Velocity);
            var diagonalDistance = (lateralWandering.LatitudeExceeded
                - vehicleHalfWidth + sourceLane.Width / 2) / Math.Sin(rotateRadian); 
            
            if (lateralWandering.WanderDirection == Side.LEFT)
                originRot -= new Vector3(0, rotateRadian * 180 / (float)Math.PI, 0);
            else 
                originRot += new Vector3(0, rotateRadian * 180 / (float)Math.PI, 0);
            
            var normalizedDirection = Quaternion.Euler(originRot) * Vector3.forward;

            var waypoints = new List<Vector3>();
            waypoints.Add(sourceLane.Waypoints[sourceWaypointId] + (normalizedDirection * (float)diagonalDistance));
            waypoints.Add(waypoints[0] + (vehDirection.normalized * lateralWandering.LongitudeDistance));
            
            var longitudeProjected = diagonalDistance * Math.Cos(rotateRadian);
              
            for (int id = sourceWaypointId; id < sourceLane.Waypoints.Length - 1; id++)
            {
                var startWp = sourceLane.Waypoints[id];
                var endWp = sourceLane.Waypoints[id + 1];
                if (CustomSimUtils.ProjectionOnLine(waypoints[1], startWp, endWp))
                {
                    var projectedPoint = Vector3.Project(waypoints[1] - startWp, endWp - startWp) + startWp;
                    var newWp = CustomSimUtils.CalculatePosition(sourceLane, projectedPoint, (float)longitudeProjected,
                        out int returnWpId, id + 1);
                    if (newWp == Vector3.zero)
                    {
                        if (nextLane == null)
                            throw new InvalidScriptException("Longitudinal wandering exceeds the ending route.");
                        var remainDistance = longitudeProjected - 
                                         CustomSimUtils.DistanceToEndingLane(sourceLane, projectedPoint, id + 1);
                        
                        newWp = CustomSimUtils.CalculatePosition(nextLane, (float)remainDistance, out int returnWpId2);
                        
                        // source lane: remove waypoints after wp[sourceWaypointId] and
                        // add two new wandering waypoints
                        var sourceWps = new List<Vector3>(sourceLane.Waypoints);
                        sourceWps.RemoveRange(sourceWaypointId + 1, 
                            sourceLane.Waypoints.Length - sourceWaypointId - 1);
                        sourceWps.AddRange(waypoints);
                        sourceLane.UpdateWaypoints(sourceWps.ToArray());
                        
                        // returning lane: add a returning waypoint (on lane), and
                        // remove redundant waypoints before it
                        var returningWps = new List<Vector3>(nextLane.Waypoints);
                        returningWps.RemoveRange(0, returnWpId2);
                        returningWps.Insert(0, newWp);
                        nextLane.UpdateWaypoints(returningWps.ToArray());
                        
                        lateralWandering.ReturningLane = nextLane.name;
                        lateralWandering.ReturningLaneWaypointIndex = returnWpId2;
                    }
                    else
                    {
                        // remove waypoints between sourceWaypointId + 1 and returnWpId
                        // add two new wandering waypoints and the returning waypoint (on lane)
                        var sourceWps = new List<Vector3>(sourceLane.Waypoints);
                        sourceWps.RemoveRange(sourceWaypointId + 1,
                            returnWpId - sourceWaypointId - 1);
                        sourceWps.Insert(sourceWaypointId + 1, newWp);
                        sourceWps.InsertRange(sourceWaypointId + 1, waypoints);
                        sourceLane.UpdateWaypoints(sourceWps.ToArray());

                        lateralWandering.ReturningLane = sourceLane.name;
                        lateralWandering.ReturningLaneWaypointIndex = sourceWaypointId + 3;
                    }
                    break;
                }
            }
            
        }
        
        // Add new waypoint to the cloned source lane to make the smooth turn,
        // and remove redundant existing waypoints behind
        private int AddUTurnWaypoints(ref TrafficLane sourceLaneClone, int sourceWaypointId,
            ref TrafficLane nextLane,
            float wheelBase, float turningWheelAngle, bool rightUTurnSide = true)
        {
            Vector3 startPoint = sourceLaneClone.Waypoints[sourceWaypointId];
            float turningRadius = wheelBase / Mathf.Sin(turningWheelAngle * Mathf.PI / 180) - 0.5f;
            int direction = rightUTurnSide ? 1 : -1;

            var tempVec = 
                ((sourceLaneClone.Waypoints[sourceWaypointId - 1] - startPoint).normalized)
                * turningRadius;
            Vector3 turningCenter = CustomSimUtils.RotatePointAroundPivot(
                tempVec + startPoint,
                startPoint,
                (90 - turningWheelAngle) * -direction);
            
            var waypoints = sourceLaneClone.Waypoints.ToList();
            
            Vector3 turningFinishPoint = turningCenter + turningCenter - startPoint;
            
            // remove redundant waypoints
            waypoints.RemoveRange(sourceWaypointId + 1, waypoints.Count - sourceWaypointId - 1);
            
            // add new waypoints to make a turn
            waypoints.Add(CustomSimUtils.RotatePointAroundPivot(startPoint, turningCenter, 30 * direction));
            waypoints.Add(CustomSimUtils.RotatePointAroundPivot(startPoint, turningCenter, 60 * direction));
            waypoints.Add(CustomSimUtils.RotatePointAroundPivot(startPoint, turningCenter, 90 * direction));
            waypoints.Add(CustomSimUtils.RotatePointAroundPivot(startPoint, turningCenter, 120 * direction));
            waypoints.Add(CustomSimUtils.RotatePointAroundPivot(startPoint, turningCenter, 150 * direction));
            waypoints.Add(turningFinishPoint);
            
            CustomSimUtils.LateralDistance(turningFinishPoint, nextLane, out int nextWaypointId);
            // Vector3 tempP = CustomSimUtils.ProjectPointOnLine(
                // turningFinishPoint, nextLane.Waypoints[nextWaypointId], nextLane.Waypoints[nextWaypointId + 1]);
            // waypoints.Add((tempP + nextLane.Waypoints[nextWaypointId + 1]) / 2);
            
            sourceLaneClone.UpdateWaypoints(waypoints.ToArray());

            return nextWaypointId;
        }
    }
}
