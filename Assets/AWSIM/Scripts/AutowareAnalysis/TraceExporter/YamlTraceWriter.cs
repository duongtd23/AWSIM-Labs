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

        public YamlTraceWriter(string filePath, GameObject autowareEgoCar, Camera sensorCamera,
            PerceptionMode perceptionMode, TraceCaptureConfig config)
            : base(filePath, autowareEgoCar, sensorCamera, perceptionMode, config)
        {
            _serializer = new SerializerBuilder().WithIndentedSequences().Build();
            _contents = "states:\n";
        }
        
        protected override void WriteFile()
        {
            // write remaining states
            var temp =
                _traceObject.states.Skip(Math.Max(0, _traceObject.states.Count - MAX_LAG_FIXED_STEPS + 1));
            _contents += _serializer.Serialize(temp);
            
            // write ego and NPC details
            DumpVehicleDetails();
            
            TraceObjectWithoutState temp2 = new TraceObjectWithoutState()
            {
                fixedTimestep = _traceObject.fixedTimestep,
                camera_screen_height = _traceObject.camera_screen_height,
                camera_screen_width = _traceObject.camera_screen_width,
                ego_detail = _traceObject.ego_detail,
                npcs_detail = _traceObject.npcs_detail,
            };
            _contents += _serializer.Serialize(temp2);
            
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