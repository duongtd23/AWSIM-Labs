using System.IO;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class ConfigLoader
    {
        private const string CONFIG_FILE = "AWAnalysis-config.json";

        public static AWAnalysisConfig Config()
        {
            if (config == null)
                config = ConfigLoader.Load();
            return config;
        }
        private static AWAnalysisConfig config;
        private static AWAnalysisConfig Load()
        {
            Debug.Log("Loading config file");
            string content = File.ReadAllText(CONFIG_FILE);
            AWAnalysisConfig config = JsonUtility.FromJson<AWAnalysisConfig>(content);
            return config;
        }
    }
}