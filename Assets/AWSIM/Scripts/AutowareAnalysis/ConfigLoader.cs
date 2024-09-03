using System.IO;
using UnityEngine;
using YamlDotNet.Serialization;

namespace AWSIM.AWAnalysis
{
    public class ConfigLoader
    {
        private const string YAML_CONFIG_FILE = "AWAnalysis-config.yaml";
        private const string YAML_CONFIG_FILE2 = "Assets/AWAnalysis-config.yaml";

        public static AWAnalysisConfig Config()
        {
            if (_config == null)
                _config = ConfigLoader.Load();
            return _config;
        }
        private static AWAnalysisConfig _config;
        private static AWAnalysisConfig Load()
        {
            string filePath =
                File.Exists(YAML_CONFIG_FILE) ? YAML_CONFIG_FILE : YAML_CONFIG_FILE2;
            if (File.Exists(filePath))
            {
                Debug.Log("Loading config file");
                var deserializer = new DeserializerBuilder().Build();
                var config = deserializer.Deserialize<AWAnalysisConfig>(File.OpenText(filePath));
                return config;
            }
            Debug.LogError($"No config file found at {YAML_CONFIG_FILE}. Use default config.");
            return new AWAnalysisConfig();
        }
    }
}