using System;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.AWAnalysis.Error;

namespace AWSIM.AWAnalysis
{
    public class CommandLineArgsManager
    {
        // all arguments
        public const string SCRIPT_ARG = "-script";
        public const string TRACE_SAVING_PATH_ARG = "-output";
        public const string PERCEPTION_ANALYSIS_ENABLE_ARG = "-perception_analysis";

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
                else if (arguments[i].StartsWith(PERCEPTION_ANALYSIS_ENABLE_ARG))
                {
                    string enableValue = ExtractArgValue(arguments, ref i, PERCEPTION_ANALYSIS_ENABLE_ARG);
                    args.Add(PERCEPTION_ANALYSIS_ENABLE_ARG, enableValue);
                }
                else if (arguments[i].StartsWith(TRACE_SAVING_PATH_ARG))
                {
                    string scriptValue = ExtractArgValue(arguments, ref i, TRACE_SAVING_PATH_ARG);
                    args.Add(TRACE_SAVING_PATH_ARG, scriptValue);
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
                    throw new CustomSimException("Value of " + 
                        argName + " argument is not provided.");
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
                else
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
            else 
            {
                scriptFilePath = _args[SCRIPT_ARG];
                return true;            
            }
        }

        public static bool GetPerceptionAnalysisFlag(out bool enabled)
        {
            Dictionary<string,string> _args = Instance().args;
            if (!_args.ContainsKey(PERCEPTION_ANALYSIS_ENABLE_ARG))
            {
                enabled = false;
                return false;
            }
            else 
            {
                string flag = _args[PERCEPTION_ANALYSIS_ENABLE_ARG];
                if (flag.ToLower() == "true")
                    enabled = true;
                else if (flag.ToLower() == "false")
                    enabled = false;
                else
                    throw new CustomSimException("Cannot parse value of argument -perception_analysis: " + flag);
                return true;            
            }
        }

        public static bool GetTraceSavingPathArg(out string outputFilePath)
        {
            Dictionary<string, string> _args = Instance().args;
            if (!_args.ContainsKey(TRACE_SAVING_PATH_ARG))
            {
                outputFilePath = "";
                return false;
            }
            else
            {
                outputFilePath = _args[TRACE_SAVING_PATH_ARG];
                return true;
            }
        }
    }
}