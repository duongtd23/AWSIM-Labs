using System.Collections;
using System.Collections.Generic;
using AWSIM;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class EgoSingletonInstance : MonoBehaviour
    {
        public GameObject autowareEgoCar;
        private Vehicle _egoVehicle;
        private CustomEgoSetting _customEgoSetting;

        public static EgoSingletonInstance Instance { get; private set; }
        
        private void Awake() 
        { 
            // If there is an instance, and it's not me, delete myself.
            if (Instance != null && Instance != this) 
            { 
                Destroy(this); 
            } 
            else 
            { 
                Instance = this;
                _egoVehicle = autowareEgoCar.GetComponent<Vehicle>();
            } 
        }

        public static GameObject AutowareEgoCarGameObject => Instance.autowareEgoCar;

        public static Vehicle AutowareEgoCarVehicle => Instance._egoVehicle;

        public static EgoDetailObject GetEgoDetailInfo()
        {
            var gameObj = AutowareEgoCarGameObject;
            MeshFilter meshFilter = gameObj.GetComponentInChildren<MeshFilter>();
            return new EgoDetailObject()
            {
                center = new Vector3Object(meshFilter.mesh.bounds.center + meshFilter.transform.parent.parent.localPosition),
                extents = new Vector3Object(meshFilter.mesh.bounds.extents)
            };
        }

        public static EgoDetailObject GetFixedEgoDetailInfo()
        {
            return new EgoDetailObject()
            {
                center = new Vector3Object(0, 0.973394155502319, 1.42438745498657),
                extents = new Vector3Object(1.09320676326752, 0.710384964942932, 2.44304156303406)
            };
        }

        public static CustomEgoSetting GetCustomEgoSetting()
        {
            return Instance?._customEgoSetting;
        }
        
        public static void SetCustomEgoSetting(CustomEgoSetting customEgoSetting)
        {
            while (Instance == null) ;
            Instance._customEgoSetting = customEgoSetting;
        }

        public static float DesiredMaxVelocity()
        {
            if (Instance._customEgoSetting == null)
                return 0;
            return Instance._customEgoSetting.EgoSettings.MaxVelocity;
        }

        public static bool ReachMaxSpeed()
        {
            return Instance._egoVehicle.Velocity.magnitude - DesiredMaxVelocity() >= -0.2f;
        }
    }
}