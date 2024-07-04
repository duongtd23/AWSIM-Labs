//using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public class NPCSpawning : MonoBehaviour
    {
        public GameObject autowareEgoCar, npcTaxi;
        private List<NPCVehicle> npcs;


        // Start is called before the first frame update
        void Start()
        {
            //vehicle = SpawnNPC();

            //TrafficLane lane = NPCSimUtils.EstimateTrafficLane(autowareEgoCar.transform, lanes);
            //Debug.Log(lane);
            //Debug.Log(lane.name);

            //SpawnOnLane("TrafficLane.448", 10);

            npcs = new List<NPCVehicle>();
            Scenario1();
        }

        //Update is called once per frame
        void FixedUpdate()
        {
            //Transform current = vehicle.gameObject.transform;
            //Vector3 newPosition = NPCSimUtils.AheadPosition(
            //    current, 0.1f);
            //vehicle.SetPosition(newPosition);

            npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);
        }

        private NPCVehicle SpawnNPC()
        {
            Vector3 spawnPoint = NPCSimUtils.AheadPosition(
                autowareEgoCar.transform, 7);

            GameObject npc = Instantiate(npcTaxi,
                spawnPoint,
                autowareEgoCar.transform.rotation);
            npc.name = "NPC-Taxi";
            var vehicle = npc.GetComponent<NPCVehicle>();
            vehicle.VehicleID = SpawnIdGenerator.Generate();
            return vehicle;
        }

        private void SpawnNPCOnLane(string trafficLaneName, float distance)
        {
            Quaternion direction;
            Vector3 spawnPoint = NPCSimUtils.PoseOnLane(trafficLaneName, distance, out direction);
            GameObject npc = Instantiate(npcTaxi,
                spawnPoint,
                direction);
        }

        private NPCVehicleSimulator npcVehicleSimulator;
        private NPCVehicleSpawner npcVehicleSpawner;
        [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        private LayerMask vehicleLayerMask;
        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        private LayerMask groundLayerMask;

        private void SpawnOnLane(string trafficLaneName, float distance)
        {
            NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();
            vehicleConfig.Acceleration = 7f;
            vehicleConfig.Deceleration = 5f;
            npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, 10, autowareEgoCar);

            npcVehicleSpawner = new NPCVehicleSpawner(this.gameObject,
                new GameObject[] { npcTaxi }, new TrafficLane[] { });

            TrafficLane lane = GameObject.Find(trafficLaneName).GetComponent<TrafficLane>();
            int waypointIndex = -1;
            Vector3 position = NPCSimUtils.CalculatePosition(lane, distance, out waypointIndex);

            NPCVehicleSpawnPoint spawnPoint = new NPCVehicleSpawnPoint(lane, position, waypointIndex);
            //NPCVehicleSpawnPoint spawnPoint = new NPCVehicleSpawnPoint(lane, 0);

            var vehicle = npcVehicleSpawner.Spawn(npcTaxi, SpawnIdGenerator.Generate(), spawnPoint);
            var routes = new List<TrafficLane> { lane, lane.NextLanes[0] };
            npcVehicleSimulator.Register(vehicle, routes, waypointIndex);
        }

        // a scenario
        private void Scenario1()
        {
            // initial setup
            NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();
            vehicleConfig.Acceleration = 7f;
            vehicleConfig.Deceleration = 8f;
            npcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, 10, autowareEgoCar);
            npcVehicleSpawner = new NPCVehicleSpawner(this.gameObject,
                new GameObject[] { npcTaxi }, new TrafficLane[] { });

            // finding lanes
            TrafficLane trafficLane_239 = GameObject.Find("TrafficLane.239").GetComponent<TrafficLane>();
            TrafficLane trafficLane_448 = GameObject.Find("TrafficLane.448").GetComponent<TrafficLane>();
            TrafficLane trafficLane_263 = GameObject.Find("TrafficLane.263").GetComponent<TrafficLane>();
            // desired speeds, defined for each lane
            var desiredSpeed = new Dictionary<string, float>()
            {
                { trafficLane_448.name, 20f },
                { trafficLane_263.name, 7f },
            };

            // set initial position on lane 239, 15m from the begining of the lane
            Vector3 position = NPCSimUtils.CalculatePosition(trafficLane_239, 15f, out int waypointIndex);
            NPCVehicleSpawnPoint spawnPoint = new NPCVehicleSpawnPoint(trafficLane_239, position, waypointIndex);

            // spawn NPC
            var npc = npcVehicleSpawner.Spawn(npcTaxi, SpawnIdGenerator.Generate(), spawnPoint);

            // set route, i.e., from lane 239 go straight to lane 448, and then change to lane 263
            var routes = new List<TrafficLane> { trafficLane_239, trafficLane_448, trafficLane_263 };

            // set goal
            // stop on lane 263, 40m far from the starting point of the lane
            var goal = new KeyValuePair<string, float>("TrafficLane.263", 40f);

            npcVehicleSimulator.Register(npc, routes, waypointIndex, desiredSpeed, goal);
            npcs.Add(npc);
        }
    }
}
