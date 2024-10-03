using AWSIM.TrafficSimulation;
using UnityEngine;

namespace AWSIM.AWAnalysis.CustomSim
{
    public class CustomNPCSpawningInterface : MonoBehaviour
    {
        public GameObject trafficLanesParent;

        // taxi, hatchback, small car, truck, van prefabs, respectively
        public GameObject npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan;
        public GameObject casualPedestrian, elegantPedestrian;

        [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        private LayerMask vehicleLayerMask;
        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        private LayerMask groundLayerMask;

        // Start is called before the first frame update
        void Start()
        {
            var autowareEgoCar = EgoSingletonInstance.AutowareEgoCarGameObject;
            CustomNPCSpawningManager.Initialize(this.gameObject,
                trafficLanesParent.GetComponentsInChildren<TrafficLane>(),
                autowareEgoCar, npcTaxi, npcHatchback,
                npcSmallCar, npcTruck, npcVan,
                casualPedestrian,elegantPedestrian,
                vehicleLayerMask, groundLayerMask);
        }

        void FixedUpdate()
        {
            // this must be enable even no scenario is specified
            CustomNPCSpawningManager.Manager()?.UpdateNPCs();
        }
    }
}