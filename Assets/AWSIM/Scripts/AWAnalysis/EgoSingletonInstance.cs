using System.Collections;
using System.Collections.Generic;
using System.Linq;
using AWSIM;
using AWSIM.AWAnalysis.CustomSim;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using AWSIM.Scripts.Vehicles.VPP_Integration;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class EgoSingletonInstance
    {
        private GameObject _autowareEgoCar;
        private AutowareVPPAdapter _egoAdapter;
        private CustomEgoSetting _customEgoSetting;

        private static EgoSingletonInstance _instance;
        public static EgoSingletonInstance Instance
        {
            get
            {
                if (_instance == null)
                    _instance = new EgoSingletonInstance();
                return _instance;
            }
        }

        public static void SetEgo(GameObject autowareEgoCar) 
        { 
            Instance._autowareEgoCar = autowareEgoCar;
            Instance._egoAdapter = autowareEgoCar.GetComponent<AutowareVPPAdapter>();
        }

        private static GameObject TryGetEgoCar()
        {
            var car = GameObject.FindWithTag("Ego");
            if (car == null) return null;
            SetEgo(car);
            return car;
        }

        public static GameObject AutowareEgoCarGameObject => 
            Instance._autowareEgoCar == null ? TryGetEgoCar():Instance._autowareEgoCar;
        public static AutowareVPPAdapter AutowareEgoCarAdapter => Instance._egoAdapter;
        public static CustomEgoSetting CustomEgoSetting => Instance._customEgoSetting;

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

        public static Camera GetObjectDetectionCamera()
        {
            var cameraobj = GameObject.FindGameObjectsWithTag("ObjectDetectionCamera").FirstOrDefault();
            if (cameraobj == null) return null;
            return cameraobj.GetComponent<Camera>();
        }
        
        public static void SetCustomEgoSetting(CustomEgoSetting customEgoSetting)
        {
            Instance._customEgoSetting = customEgoSetting;
        }

        public static float DesiredMaxVelocity()
        {
            if (Instance._customEgoSetting?.EgoSettings == null || 
                Instance._customEgoSetting.EgoSettings.MaxVelocity == 0)
                return ConfigLoader.Config().EgoDefaultVelocity;
            return Instance._customEgoSetting.EgoSettings.MaxVelocity;
        }

        public static bool ReachMaxSpeed()
        {
            return Instance._egoAdapter.Velocity.magnitude - DesiredMaxVelocity() >= -0.1f;
        }
    }
}