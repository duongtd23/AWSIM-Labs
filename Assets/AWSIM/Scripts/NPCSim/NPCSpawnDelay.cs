﻿using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public enum NPCDelayType
    {
        FROM_BEGINNING,           //default
        UNTIL_EGO_GOT_TRAJECTORY,
        UNTIL_EGO_MOVE
    }
    public class NPCSpawnDelay
    {
        public float DelayAmount { get; set; }

        public NPCDelayType DelayType { get; set; }

        // Delay `delay` seconds from the beginning
        public static NPCSpawnDelay Delay(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = NPCDelayType.FROM_BEGINNING,
            };
        }
        // Delay until the Ego vehicle got trajectory (in seconds).
        // E.g., if the passed param (`delay`) is 2,
        // it will be delayed 2 seconds after the Ego vehicle got trajectory.
        // If `delay` is 0, the action concerned will be triggered at the same time
        // when the Ego got planning trajectory.
        public static NPCSpawnDelay DelayUntilEgoGotTrajectory(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = NPCDelayType.UNTIL_EGO_GOT_TRAJECTORY,
            };
        }
        // Delay until the Ego vehicle moves (in seconds)
        // E.g., if the passed param (`delay`) is 2,
        // it will be delayed 2 seconds after the Ego vehicle moves.
        // Don't set `delay` value to 0 as it may cause the NPC and the Ego never move.
        // In such a case, use DelayUntilEgoGotTrajectory instead
        public static NPCSpawnDelay DelayUntilEgoMove(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = NPCDelayType.UNTIL_EGO_MOVE,
            };
        }
    }

    public class DelayingNPCVehicle
    {
        public uint VehicleID { get; set; }
        public GameObject VehiclePrefab { get; set; }
        public LanePosition SpawnPosition { get; set; }
        public List<string> Route { get; set; }
        public Dictionary<string, float> DesiredSpeeds { get; set; }
        public LanePosition Goal { get; set; }
        public NPCSpawnDelay Delay { get; set; }
        public DelayingNPCVehicle(uint vehicleID, GameObject vehiclePrefab, LanePosition spawnPosition, List<string> route,
            Dictionary<string, float> desiredSpeeds, LanePosition goal, NPCSpawnDelay delay)
        {
            VehicleID = vehicleID;
            VehiclePrefab = vehiclePrefab;
            SpawnPosition = spawnPosition;
            Route = route;
            DesiredSpeeds = desiredSpeeds;
            Goal = goal;
            Delay = delay;
        }
    }
}