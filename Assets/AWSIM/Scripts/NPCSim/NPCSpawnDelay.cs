using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public class NPCSpawnDelay
    {
        public float DelayAmount { get; set; }

        public bool UntilEgoMove { get; set; }

        // delay to spawn (in seconds)
        public static NPCSpawnDelay Delay(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay
            };
        }
        // delay until the Ego vehicle moves (in seconds)
        // E.g., if the passed param (`delay`) is 2,
        // it will be delayed 2 seconds after the Ego vehicle moves
        // if you want to spawn an NPC at the same time the Ego start moving, set this param to 0
        public static NPCSpawnDelay DelayUntilEgoMove(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                UntilEgoMove = true
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