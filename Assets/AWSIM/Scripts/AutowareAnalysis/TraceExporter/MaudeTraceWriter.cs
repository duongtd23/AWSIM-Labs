using System.IO;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class MaudeTraceWriter : TraceWriter
    {
        public const string MAUDE_TEMPLATE = "in ../base.maude\n\nmod TRACE is " +
                                             "\n  pr FORMULAS .\n\n";
        private string _contents;

        public MaudeTraceWriter(string filePath, GameObject autowareEgoCar, Camera sensorCamera,
            PerceptionMode perceptionMode, TraceCaptureConfig config)
            : base(filePath, autowareEgoCar, sensorCamera, perceptionMode, config)
        {
            _contents = MAUDE_TEMPLATE + 
                        "  eq init = ";
        }

        protected override void WriteFile()
        {
            // write remaining states
            int numberOfState = _traceObject.states.Count;
            string stateStr = "";
            for (int i = numberOfState - MAX_LAG_FIXED_STEPS + 1; i < numberOfState; i++)
            {
                stateStr = _traceObject.states[i].DumpMaudeStr();
                _contents += $"{stateStr} .\n  rl  {stateStr}\n  =>  ";
            }
            _contents += $"{stateStr} .\n";
            
            // write ego and NPC details
            DumpVehicleDetails();

            _contents += $"  eq fixedTimestep = {(int)(Time.fixedDeltaTime * 1000)} .\n" +
                         $"  eq cameraScreenWidth = {_sensorCamera.pixelWidth} .\n" +
                         $"  eq cameraScreenHeight = {_sensorCamera.pixelHeight} .\n" +
                         $"  eq boundsCenter(\"ego\") = {_traceObject.ego_detail.center.DumpMaudeStr()} .\n" +
                         $"  eq boundsExtent(\"ego\") = {_traceObject.ego_detail.extents.DumpMaudeStr()} .\n";
            foreach (var npc in _traceObject.npcs_detail)
            {
                _contents += $"  eq boundsCenter(\"{npc.name}\") = {npc.center.DumpMaudeStr()} .\n" +
                             $"  eq boundsExtent(\"{npc.name}\") = {npc.extents.DumpMaudeStr()} .\n";
            }
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