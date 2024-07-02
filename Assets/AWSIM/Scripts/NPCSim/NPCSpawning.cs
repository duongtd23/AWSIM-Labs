using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    public class NPCSpawning : MonoBehaviour
    {
        public GameObject autowareEgoCar, npcTaxi;
        private NPCVehicle vehicle;


        // Start is called before the first frame update
        void Start()
        {
            vehicle = SpawnNPC();
        }

        // suppose that the vehicle has a constant speed
        void FixedUpdate()
        {
            Transform current = vehicle.gameObject.transform;
            Vector3 newPosition = NPCSimUtils.AheadPosition(
                current, 0.1f);
            vehicle.SetPosition(newPosition);
        }

        private NPCVehicle SpawnNPC()
        {
            Vector3 spawnPoint = NPCSimUtils.AheadPosition(
                autowareEgoCar.transform, 7);

            GameObject npc = Object.Instantiate(npcTaxi,
                spawnPoint,
                autowareEgoCar.transform.rotation);
            npc.name = "NPC-Taxi";
            var vehicle = npc.GetComponent<NPCVehicle>();
            vehicle.VehicleID = SpawnIdGenerator.Generate();
            return vehicle;
        }
    }
}
