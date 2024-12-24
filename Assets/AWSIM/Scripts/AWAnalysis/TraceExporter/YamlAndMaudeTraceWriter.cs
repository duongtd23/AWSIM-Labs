using System;
using System.IO;
using System.Linq;
using AWSIM_Script.Object;
using AWSIM.AWAnalysis.TraceExporter.Objects;
using YamlDotNet.Serialization;
using UnityEngine;

namespace AWSIM.AWAnalysis.TraceExporter
{
    public class YamlAndMaudeTraceWriter : TraceWriter
    {
        private string _yamlcontents;
        private ISerializer _yamlserializer;

        private string _maudecontents;
        
        public YamlAndMaudeTraceWriter(string filePath, Camera sensorCamera,
            PerceptionMode perceptionMode, TraceCaptureConfig config)
            : base(filePath, sensorCamera, perceptionMode, config)
        {
            // yaml
            _yamlserializer = new SerializerBuilder().WithIndentedSequences().Build();
            _yamlcontents = "states:\n";
            
            // maude
            _maudecontents = MaudeTraceWriter.MAUDE_TEMPLATE;
            _maudecontents += "\n  eq init = ";
        }
        
        protected override void WriteFile()
        {
            if (!ValidateFilePath()) return;

            // write remaining states
            // yaml
            var remainStates =
                _traceObject.states.Skip(Math.Max(0, _traceObject.states.Count - MAX_LAG_FIXED_STEPS + 1));
            _yamlcontents += _yamlserializer.Serialize(remainStates);
            
            // maude
            string stateStr = "";
            foreach (var state in remainStates)
            {
                stateStr = state.DumpMaudeStr();
                _maudecontents += $"{stateStr} .\n  rl  {stateStr}\n  =>  ";
            }
            _maudecontents += $"{stateStr} .\n";
            if (ConfigLoader.Config().MaudeTraceImportFile?.Trim() != string.Empty)
                _maudecontents = ConfigLoader.Config().MaudeTraceImportFile + "\n\n" + _maudecontents;

            
            // write ego and NPC details
            DumpVehicleDetails();

            TraceObjectWithoutState temp2 = _traceObject.Clone();
            _yamlcontents += _yamlserializer.Serialize(temp2);
            
            File.WriteAllText(_filePath + ".yaml", _yamlcontents);
            
            // maude
            _maudecontents += $"  eq fixedTimestep = {(int)(Time.fixedDeltaTime * 1000)} .\n" +
                         $"  eq cameraScreenWidth = {_sensorCamera.pixelWidth} .\n" +
                         $"  eq cameraScreenHeight = {_sensorCamera.pixelHeight} .\n" +
                         $"  eq boundsCenter(\"ego\") = {_traceObject.ego_detail.center.DumpMaudeStr()} .\n" +
                         $"  eq boundsExtent(\"ego\") = {_traceObject.ego_detail.extents.DumpMaudeStr()} .\n";
            foreach (var npc in _traceObject.npcs_detail)
            {
                _maudecontents += $"  eq boundsCenter(\"{npc.name}\") = {npc.center.DumpMaudeStr()} .\n" +
                             $"  eq boundsExtent(\"{npc.name}\") = {npc.extents.DumpMaudeStr()} .\n";
            }
            
            // write cut-in cut-out info
            if (_traceObject.other is CutInInfoObject cutInInfoObject)
            {
                _maudecontents += $"  eq cutinNPC = \"{cutInInfoObject.cutin_npc_name}\" .\n";
                _maudecontents += $"  eq cutinStartTime = {cutInInfoObject.time_cutin_start} .\n";
            }
            else if (_traceObject.other is CutOutInfoObject cutOutInfoObject)
            {
                _maudecontents += $"  eq cutoutNPC = \"{cutOutInfoObject.cutout_npc_name}\" .\n";
                _maudecontents += $"  eq cutoutStartTime = {cutOutInfoObject.time_cutout_start} .\n";
            }
            else if (_traceObject.other is DecelerationInfoObject decelInfoObject)
            {
                _maudecontents += $"  eq decelerationNPC = \"{decelInfoObject.deceleration_npc_name}\" .\n";
                _maudecontents += $"  eq decelerationStartTime = {decelInfoObject.time_deceleration_start} .\n";
            }
            else if (_traceObject.other is SwerveInfoObject swerveInfoObject)
            {
                _maudecontents += $"  eq swerveNPC = \"{swerveInfoObject.swerve_npc_name}\" .\n";
                _maudecontents += $"  eq swerveStartTime = {swerveInfoObject.time_swerve_start} .\n";
            }
            else if (_traceObject.other is UTurnInfoObject uturnInfoObject)
            {
                _maudecontents += $"  eq uturnNPC = \"{uturnInfoObject.uturn_npc_name}\" .\n";
                _maudecontents += $"  eq uturnStartTime = {uturnInfoObject.time_uturn_start} .\n";
            }

            _maudecontents += "endm";
            
            if (!string.IsNullOrEmpty(_traceObject.comment))
                _maudecontents += $"\n--- {_traceObject.comment}";

            File.WriteAllText(_filePath + ".maude", _maudecontents);
        }
        
        protected override void UpdateTraceObject(double timeStamp)
        {
            base.UpdateTraceObject(timeStamp);
            int i = _traceObject.states.Count - MAX_LAG_FIXED_STEPS;
            if (i >= 0)
            {
                // dump the state
                // yaml
                _yamlcontents += _yamlserializer.Serialize(new StateObject[1]{_traceObject.states[i]});
                
                // maude
                string stateStr = _traceObject.states[i].DumpMaudeStr();
                _maudecontents += $"{stateStr} .\n  rl  {stateStr}\n  =>  ";
            }
        }
    }
}