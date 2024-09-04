using System.IO;
using UnityEngine;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class MaudeTraceWriter : TraceWriter
    {
        public const string MAUDE_TEMPLATE = "in ../base.maude\n\nmod TRACE is " +
                                             "\n  pr FORMULAS .\n\n";
        private string _contents;

        public MaudeTraceWriter(string filePath, Vehicle egoVehicle, Camera sensorCamera,
            PerceptionMode perceptionMode)
            : base(filePath, egoVehicle, sensorCamera, perceptionMode)
        {
            _contents = MAUDE_TEMPLATE;
            _contents += $"  eq fixedTimestep = {(int)(Time.fixedDeltaTime * 1000)} .\n" +
                         $"  eq init = ";
        }

        protected override void WriteFile()
        {
            int numberOfState = _traceObject.states.Count;
            string stateStr = "";
            for (int i = numberOfState - MAX_LAG_FIXED_STEPS + 1; i < numberOfState; i++)
            {
                stateStr = _traceObject.states[i].DumpMaudeStr();
                _contents += $"{stateStr} .\n  rl  {stateStr}\n  =>  ";
            }

            _contents += $"{stateStr} .\n";
            _contents += $"  eq cameraScreenWidth = {_sensorCamera.pixelWidth} .\n" +
                         $"  eq cameraScreenHeight = {_sensorCamera.pixelHeight} .\n";
            _contents += "endm";
            File.WriteAllText(_filePath, _contents);
        }

        protected override void UpdateTraceObject(double timeStamp)
        {
            base.UpdateTraceObject(timeStamp);
            int i = _traceObject.states.Count - MAX_LAG_FIXED_STEPS;
            if (i >= 0)
            {
                // dump the state
                string stateStr = _traceObject.states[i].DumpMaudeStr();
                _contents += $"{stateStr} .\n  rl  {stateStr}\n  =>  ";
            }
        }
    }
}