using System.IO;
using UnityEngine;
using YamlDotNet.Serialization;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class YamlTraceWriter : TraceWriter
    {
        public YamlTraceWriter(string filePath, Vehicle egoVehicle, Camera sensorCamera,
            PerceptionMode perceptionMode)
            : base(filePath, egoVehicle, sensorCamera, perceptionMode)
        { }
        
        protected override void WriteFile()
        {
            var serializer = new SerializerBuilder().Build();
            var yaml = serializer.Serialize(_traceObject);
            File.WriteAllText(_filePath, yaml);
        }
    }
}