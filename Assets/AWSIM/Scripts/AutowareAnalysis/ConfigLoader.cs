using System.IO;
using UnityEngine;

namespace AWSIM.AWAnalysis
{
    public class ConfigLoader
    {
        private const string CONFIG_FILE = "AWAnalysis-config.json";

        public static AWAnalysisConfig Config()
        {
            if (_config == null)
                _config = ConfigLoader.Load();
            return _config;
        }
        private static AWAnalysisConfig _config;
        private static AWAnalysisConfig Load()
        {
            if (File.Exists(CONFIG_FILE))
            {
                Debug.Log("Loading config file");
                string content = File.ReadAllText(CONFIG_FILE);
                AWAnalysisConfig config = JsonUtility.FromJson<AWAnalysisConfig>(content);
                return config;
            }
            Debug.LogError($"No config file found at {CONFIG_FILE}. Use default config.");
            return new AWAnalysisConfig();
        }
    }
}