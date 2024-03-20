using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Linq;

namespace AWSIM.Loader
{
static class SimConfiguration
{
    static public void Configure(EgosManager egoManager, MapManager mapManager, SimulationManager simulationManager)
    {
        //////////////////////
        // Levels load order:
        // LoaderScene -> MapScene -> SimulationScene
        //
        // Code in this section activates after all "Start()" and "Awake()" from map and simulation scenes.

        //////////////////////
        // Configuration resources

        // Get our scene handles
        Scene simulationScene = SceneManager.GetSceneByName(simulationManager.simulationSceneName);
        Scene mapScene = SceneManager.GetSceneByName(mapManager.spawnedMapName);
        
        // We set the map scene to active to get its environment parameters (skybox and ambient lighting)
        SceneManager.SetActiveScene(mapScene);

        // Get configurations
        var simulationConfiguration = simulationManager.simulationConfiguration;
        var mapConfiguration = mapManager.mapConfiguration;
        var egoConfiguration = egoManager.egoConfiguration;
        
        //////////////////////
        // Ego Spawn

        // We want our ego to spawn under the Simulation scene root object
        var ego = egoManager.SpawnEgo(egoManager.gameObject.transform);

        //////////////////////
        // Core initialization

        // Set Ego position manager
        Scripts.UI.EgoVehiclePositionManager positionManager = GameObject.FindObjectOfType<Scripts.UI.EgoVehiclePositionManager>();
        positionManager.InitializeEgoTransform(ego.transform);

        // Set scene time scale
        DemoUI demoUi = GameObject.FindObjectOfType<DemoUI>();
        demoUi.SetTimeScale(simulationConfiguration.timeScale);
        demoUi.TimeScaleSlider.value = simulationConfiguration.timeScale;

        // Set traffic on/off
        var trafficSims = GameObject.FindObjectsOfType<TrafficSimulation.TrafficManager>();
        foreach (var trafficSim in trafficSims)
        {
            trafficSim.gameObject.SetActive(simulationConfiguration.useTraffic);
        }
    }
}
}