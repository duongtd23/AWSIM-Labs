using System.IO;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using UnityEngine;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class MaudeTraceWriter : TraceWriter
    {
        public const string MAUDE_TEMPLATE = "mod TRACE is\n" +
                                             "  pr FORMULAS .\n";
        private string _contents;
        private string _statesStr;
        private readonly bool _writeStateData;

        public MaudeTraceWriter(string filePath, Camera sensorCamera,
            PerceptionMode perceptionMode, TraceCaptureConfig config)
            : base(filePath, sensorCamera, perceptionMode, config)
        {
            _writeStateData = ConfigLoader.Config().MaudeTraceWriteStateData;
            _contents = MAUDE_TEMPLATE;
            if (_writeStateData)
                _contents += "  pr STATES .\n";
            _contents += "\n  eq init = ";
            _statesStr = "mod STATES is\n" +
                         "  pr FORMULAS .\n\n" +
                         "  op states : -> List{AWState} .\n" +
                         "  eq states =";
        }

        protected override void WriteFile()
        {
            if (!ValidateFilePath()) return;
            
            // write remaining states
            int numberOfState = _traceObject.states.Count;
            string stateStr = "";
            for (int i = numberOfState - MAX_LAG_FIXED_STEPS + 1; i < numberOfState; i++)
            {
                stateStr = _traceObject.states[i].DumpMaudeStr();
                _contents += $"{stateStr} .\n  rl  {stateStr}\n  =>  ";
                if (_writeStateData)
                {
                    _statesStr += $"  ({stateStr})\n";
                }
            }
            _contents += $"{stateStr} .\n";

            if (_writeStateData)
            {
                _statesStr += "  .\nendm\n\n";
                _contents = _statesStr + _contents + "\n";
            }

            _contents = ConfigLoader.Config().MaudeTraceImportFile + "\n\n" + _contents;
            
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
            
            // write cut-in cut-out info
            if (_traceObject.other is CutInInfoObject cutInInfoObject)
            {
                _contents += $"  eq cutinNPC = \"{cutInInfoObject.cutin_npc_name}\" .\n";
                _contents += $"  eq cutinStartTime = {cutInInfoObject.time_cutin_start} .\n";
            }
            else if (_traceObject.other is CutOutInfoObject cutOutInfoObject)
            {
                _contents += $"  eq cutoutNPC = \"{cutOutInfoObject.cutout_npc_name}\" .\n";
                _contents += $"  eq cutoutStartTime = {cutOutInfoObject.time_cutout_start} .\n";
            }
            else if (_traceObject.other is DecelerationInfoObject decelInfoObject)
            {
                _contents += $"  eq decelerationNPC = \"{decelInfoObject.deceleration_npc_name}\" .\n";
                _contents += $"  eq decelerationStartTime = {decelInfoObject.time_deceleration_start} .\n";
            }

            _contents += "endm";
            
            if (!string.IsNullOrEmpty(_traceObject.comment))
                _contents += $"\n--- {_traceObject.comment}";

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
                if (_writeStateData)
                {
                    _statesStr += $"  ({stateStr})\n";
                }
            }
        }
    }
}