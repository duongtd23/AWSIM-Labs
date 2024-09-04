using System;
using System.IO;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;
using System.Linq;
using YamlDotNet.Serialization;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class YamlTraceWriter : TraceWriter
    {
        private string _contents;
        private ISerializer _serializer;

        public YamlTraceWriter(string filePath, Vehicle egoVehicle, Camera sensorCamera,
            PerceptionMode perceptionMode)
            : base(filePath, egoVehicle, sensorCamera, perceptionMode)
        {
            _serializer = new SerializerBuilder().WithIndentedSequences().Build();
            TraceObjectWithoutState temp = new TraceObjectWithoutState()
            {
                fixedTimestep = _traceObject.fixedTimestep,
                camera_screen_height = _traceObject.camera_screen_height,
                camera_screen_width = _traceObject.camera_screen_width,
            };
            _contents = _serializer.Serialize(temp);
            _contents += "states:\n";
        }
        
        protected override void WriteFile()
        {
            var temp =
                _traceObject.states.Skip(Math.Max(0, _traceObject.states.Count - MAX_LAG_FIXED_STEPS + 1));
            _contents += _serializer.Serialize(temp);

            // var yaml = _serializer.Serialize(_traceObject);
            File.WriteAllText(_filePath, _contents);
        }
        
        protected override void UpdateTraceObject(double timeStamp)
        {
            base.UpdateTraceObject(timeStamp);
            int i = _traceObject.states.Count - MAX_LAG_FIXED_STEPS;
            if (i >= 0)
            {
                // dump the state
                _contents += _serializer.Serialize(new StateObject[1]{_traceObject.states[i]});
            }
        }
    }
}