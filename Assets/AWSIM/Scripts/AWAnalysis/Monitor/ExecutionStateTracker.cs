using System;
using autoware_adapi_v1_msgs.msg;
using UnityEngine;

namespace AWSIM.AWAnalysis.Monitor
{
    public class ExecutionStateTracker
    {
        private static ExecutionStateTracker _instance;
        private ExecutionState _state = ExecutionState.LOCALIZATION_UNINITIALIZED;
        
        // time when autonomous operation mode becomes ready
        private float _autoOpModeReadyTime = Mathf.Infinity;

        /*
         * motion state from AWF
         * uint16 UNKNOWN = 0
           uint16 STOPPED = 1
           uint16 STARTING = 2
           uint16 MOVING = 3
         */
        private int _motionState;

        /*
         * routing state from AWF
         * uint16 UNKNOWN = 0
           uint16 UNSET = 1
           uint16 SET = 2
           uint16 ARRIVED = 3
           uint16 CHANGING = 4
         */
        private int _routingState;
        
        /*
         * operation state from AWF
         * # constants for mode
           uint8 UNKNOWN = 0
           uint8 STOP = 1
           uint8 AUTONOMOUS = 2
           uint8 LOCAL = 3
           uint8 REMOTE = 4
           
           # variables
           builtin_interfaces/Time stamp
           uint8 mode
           bool is_autoware_control_enabled
           bool is_in_transition
           bool is_stop_mode_available
           bool is_autonomous_mode_available
           bool is_local_mode_available
           bool is_remote_mode_available
         */
        private int _operationState;
        private bool _isAutonomousModeAvailable;
        
        public static int MotionState =>
            _instance?._motionState ?? 0;

        public static int RoutingState =>
            _instance?._routingState ?? 0;

        public static int OperationState =>
            _instance?._operationState ?? 0;
        
        public static bool IsAutonomousModeAvailable =>
            _instance?._isAutonomousModeAvailable ?? false;

        public static float AutoOpModeReadyTime => 
            _instance?._autoOpModeReadyTime ?? Mathf.Infinity;

        public static ExecutionState State => 
            _instance?._state ?? ExecutionState.LOCALIZATION_UNINITIALIZED;

        // must be invoked somewhere
        public static void Start()
        {
            _instance = new ExecutionStateTracker();
            SimulatorROS2Node.CreateSubscription<OperationModeState>(
                TopicName.TOPIC_API_OPERATION_MODE_STATE, msg =>
                {
                    _instance._operationState = msg.Mode;
                    if (msg.Is_autonomous_mode_available)
                    {
                        _instance._isAutonomousModeAvailable = true;
                        if (_instance._state == ExecutionState.LOCALIZATION_SUCCEEDED)
                        {
                            _instance._autoOpModeReadyTime = AWAnalysis.GetFixedTime();
                            _instance._state = ExecutionState.AUTO_MODE_READY;
                            Debug.Log("[AW-Analysis] Autonomous mode ready.");
                        }
                    }
                });
            SimulatorROS2Node.CreateSubscription<LocalizationInitializationState>(
                TopicName.TOPIC_LOCALIZATION_INITIALIZATION_STATE, msg =>
                {
                    if (msg.State == LocalizationInitializationState.INITIALIZED && 
                        _instance._state < ExecutionState.LOCALIZATION_SUCCEEDED)
                    {
                        _instance._state = ExecutionState.LOCALIZATION_SUCCEEDED;
                        Debug.Log("[AW-Analysis] Localization succeeded.");
                    }
                });
            SimulatorROS2Node.CreateSubscription<RouteState>(
                TopicName.TOPIC_API_ROUTING_STATE, msg =>
                {
                    _instance._routingState = msg.State;
                    if (msg.State == RouteState.ARRIVED &&
                        _instance._state < ExecutionState.GOAL_ARRIVED)
                        _instance._state = ExecutionState.GOAL_ARRIVED;
                });
            SimulatorROS2Node.CreateSubscription<MotionState>(
                "/api/motion/state", msg =>
                {
                    _instance._motionState = msg.State;
                });
        }

        public static void ResetState()
        {
            _instance._state = ExecutionState.LOCALIZATION_UNINITIALIZED;
        }
    }
    
    public enum ExecutionState
    {
        LOCALIZATION_UNINITIALIZED = 0,
        LOCALIZATION_SUCCEEDED = 1,
        AUTO_MODE_READY = 2,
        // APPROACHING_GOAL,
        GOAL_ARRIVED = 5,
    }
}