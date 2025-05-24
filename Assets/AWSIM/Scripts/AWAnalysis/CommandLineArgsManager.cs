using System;
using System.Collections.Generic;
using AWSIM_Script.Object;
using UnityEngine;
using AWSIM.AWAnalysis.Error;

namespace AWSIM.AWAnalysis
{
    public class CommandLineArgsManager
    {
        // all arguments
        public const string SCRIPT_ARG = "-script";
        public const string TRACE_SAVING_PATH_ARG = "-output";
        public const string PERCEPTION_MODE_ARG = "-perception_mode";
        public const string NOISE_CONFIG_ARG = "-noise";

        // singleton instance
        private static CommandLineArgsManager instance;

        private Dictionary<string,string> args;

        public static CommandLineArgsManager Instance()
        {
            if (instance == null)
                instance = new CommandLineArgsManager();
            return instance;
        }

        private CommandLineArgsManager()
        {
            args = new Dictionary<string, string>();
            string[] arguments = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < arguments.Length; i++)
            {
                if (arguments[i].StartsWith(SCRIPT_ARG))
                {
                    string scriptValue = ExtractArgValue(arguments, ref i, SCRIPT_ARG);
                    args.Add(SCRIPT_ARG, scriptValue);
                }
                else if (arguments[i].StartsWith(PERCEPTION_MODE_ARG))
                {
                    string mode = ExtractArgValue(arguments, ref i, PERCEPTION_MODE_ARG);
                    args.Add(PERCEPTION_MODE_ARG, mode);
                }
                else if (arguments[i].StartsWith(TRACE_SAVING_PATH_ARG))
                {
                    string filePath = ExtractArgValue(arguments, ref i, TRACE_SAVING_PATH_ARG);
                    args.Add(TRACE_SAVING_PATH_ARG, filePath);
                }
                else if (arguments[i].StartsWith(NOISE_CONFIG_ARG))
                {
                    string noiseConfig = ExtractArgValue(arguments, ref i, NOISE_CONFIG_ARG);
                    args.Add(NOISE_CONFIG_ARG, noiseConfig.ToLower());
                }
            }
        }

        private string ExtractArgValue(string[] arguments, ref int index, string argName)
        {
            // when arguments[index] is `-script /tmp/input.txt`
            if (arguments[index] == argName)
            {
                index += 1;
                if (index >= arguments.Length)
                {
                    if (argName == NOISE_CONFIG_ARG)
                        return "true";
                    throw new CustomSimException("Value of " +
                                                 argName + " argument is not provided.");
                }

                // when the arg is solely "-noise" (without true and false suffix)
                if (argName == NOISE_CONFIG_ARG && 
                    arguments[index].ToLower() != "true" && arguments[index].ToLower() != "false")
                {
                    index -= 1;
                    return "true";
                }

                return arguments[index];
            }
            // when arguments[index] is `-script=/tmp/input.txt`
            else
            {
                string remainStr = arguments[index].Substring(argName.Length);
                if (remainStr.StartsWith("="))
                {
                    return remainStr.Substring(1);
                }
                throw new CustomSimException("Cannot parse argument " + arguments[index]);
            }
        }

        public static bool GetScriptArg(out string scriptFilePath)
        {
            Dictionary<string,string> _args = Instance().args;
            if (!_args.ContainsKey(SCRIPT_ARG))
            {
                scriptFilePath = "";
                return false;
            }
            scriptFilePath = _args[SCRIPT_ARG];
            return true;            
        }

        public static PerceptionMode GetPerceptionModeArg()
        {
            Dictionary<string,string> _args = Instance().args;
            if (_args.ContainsKey(PERCEPTION_MODE_ARG) && _args[PERCEPTION_MODE_ARG].ToLower() == "camera_lidar_fusion")
                return PerceptionMode.CAMERA_LIDAR_FUSION;
            return PerceptionMode.LIDAR;
        }

        public static bool GetTraceSavingPathArg(out string outputFilePath)
        {
            Dictionary<string, string> _args = Instance().args;
            if (!_args.ContainsKey(TRACE_SAVING_PATH_ARG))
            {
                outputFilePath = "";
                return false;
            }
            outputFilePath = _args[TRACE_SAVING_PATH_ARG];
            return true;
        }
        
        public static bool GetNoiseConfigArg(out bool isNoiseEnable)
        {
            Dictionary<string, string> _args = Instance().args;
            if (!_args.ContainsKey(NOISE_CONFIG_ARG))
            {
                isNoiseEnable = true;
                return false;
            }
            isNoiseEnable = _args[NOISE_CONFIG_ARG].ToLower() == "true";
            return true;
        }


        public static float TraceSavingTimeout { get; set; } = Simulation.DUMMY_SAVING_TIMEOUT;
    }
}