using System;
using UnityEngine;
using Antlr4.Runtime.Tree;
using AWSIM_Script.Error;
using AWSIM_Script.Object;
using AWSIM_Script.Parser.Object;
using static AWSIMScriptGrammarParser;
using System.Collections.Generic;
using System.Linq;
using Antlr4.Runtime;

namespace AWSIM_Script.Parser
{
	public class ScenarioParser
	{
        public const string DELAY_SPAWN = "delay-spawn";
        public const string DELAY_MOVE = "delay-move";
        public const string DELAY_SPAWN_UNTIL_EGO_ENGAGED = "delay-spawn-until-ego-engaged";
        public const string DELAY_MOVE_UNTIL_EGO_ENGAGED = "delay-move-until-ego-engaged";
        public const string DELAY_SPAWN_UNTIL_EGO_MOVE = "delay-spawn-until-ego-move";
        public const string DELAY_MOVE_UNTIL_EGO_MOVE = "delay-move-until-ego-move";
        public const string RELATIVE_POSITION_LEFT = "left";
        public const string RELATIVE_POSITION_RIGHT = "right";
        public const string RELATIVE_POSITION_BACK = "back";
        public const string RELATIVE_POSITION_FORWARD = "forward";
        public const string AGGRESSIVE_DRIVING = "aggressive-driving";
        public const string ACCELERATION = "acceleration";
        public const string DECELERATION = "deceleration";
        public const string EGO_MAX_VELOCITY = "max-velocity";
        public const string SAVING_TIMEOUT = "saving-timeout";
        public const string CHANGE_LANE = "change-lane";
        public const string CUT_IN = "cut-in";
        public const string CUT_OUT = "cut-out";
        public const string AT = "at";
        public const string DX = "dx";
        public const string IGNORE_EXP = "_";

        
        private ScenarioScore scenarioScore;
        public ScenarioParser(ScenarioScore scenarioScore)
		{
            this.scenarioScore = scenarioScore;
		}

        public Simulation Parse()
        {
            var runFuncs = scenarioScore.Functions.FindAll(function => function.Name == FunctionParser.FUNCTION_RUN);
            if (runFuncs.Count == 0)
            {
                Debug.LogWarning("[AWAnalysis] There is no run function in the input script.");
                return new Simulation();
            }
            if (runFuncs.Count > 1)
                Debug.LogError("[AWAnalysis] Found more than run function in the input script." +
                    " Use the last one by default.");
            var runFunc = runFuncs[^1];

            if (runFunc.Parameters.Count < 1)
                throw new InvalidScriptException("Invalid arguments passed for function run: ");

            Simulation simulation = new Simulation();
            List<ExpressionContext> npcsExpContexts = new List<ExpressionContext>();
            ParserRuleContext egoFuncExp = FunctionExpContext.EmptyContext;

            // if the first param is NPC list
            if (runFunc.Parameters[0].children?[0] is ArrayExpContext)
            {
                npcsExpContexts = ParserUtils.
                ParseArray((ArrayExpContext)runFunc.Parameters[0].children[0]);
            }
            // if the first param is Ego
            else if (runFunc.Parameters[0].children?[0] is VariableExpContext)
            {
                VariableExpContext varExp = (VariableExpContext)runFunc.Parameters[0].children[0];
                if (!scenarioScore.Variables.ContainsKey(varExp.GetText()))
                    throw new InvalidScriptException("Undefined variable: " + varExp.GetText());
                ExpressionContext tempExp = scenarioScore.Variables[varExp.GetText()];
                if (tempExp.children == null ||
                    tempExp.children.Count == 0 ||
                    !(tempExp.children[0] is FunctionExpContext))
                    throw new InvalidScriptException("Expected a function defining Ego, but get: "
                        + tempExp.GetText());
                egoFuncExp = (FunctionExpContext)tempExp.children[0];
            }
            else if (runFunc.Parameters[0].children?[0] is FunctionExpContext)
            {
                egoFuncExp = (FunctionExpContext)runFunc.Parameters[0].children[0];
            }
            else
                throw new InvalidScriptException("Invalid arguments passed for function run: ");

            // if the second param is NPC list
            if (runFunc.Parameters.Count > 1)
            {
                if (runFunc.Parameters[1].children?[0] is ArrayExpContext arrayExpContext)
                {
                    npcsExpContexts = ParserUtils.ParseArray(arrayExpContext);
                }
                else
                    throw new InvalidScriptException("Expected an array of NPCs, but get: " + runFunc.Parameters[1].GetText());
            }

            // Prase Ego
            if (egoFuncExp != FunctionExpContext.EmptyContext)
                RetrieveEgo((FunctionExpContext)egoFuncExp, ref simulation);

            // Parse NPCs
            foreach (ExpressionContext npcExpContext in npcsExpContexts)
            {
                RetrieveNPC(npcExpContext, ref simulation);
            }
            
            // 3rd param (optional): simulation setting,
            // e.g., saving-timeout(30) indicates the trace will be exported at time 30s or Ego reaches goal, which ever happens first
            if (runFunc.Parameters.Count > 2)
            {
                ParseSimulationConfig(runFunc.Parameters[2].children[0], ref simulation);
            }
            
            ParsingPostProcess(ref simulation);
            
            return simulation;
        }

        private void ParsingPostProcess(ref Simulation simulation)
        {
            // assign leading vehicle for the cut-out scenario
            for (int i = 0; i < simulation.NPCs.Count; i++)
            {
                var npc = simulation.NPCs[i];
                if (npc.HasConfig() &&
                    npc.Config.HasALaneChange() &&
                    npc.Config.LaneChange is CutOutLaneChange cutOutLaneChange)
                {
                    var others = simulation.NPCs.FindAll(other => 
                        !other.Equals(npc) && 
                        !other.HasGoal() &&
                        other.InitialPosition.Equals(LaneOffsetPosition.DummyPosition()) &&
                        other.HasDelayOption() &&
                        other.SpawnDelayOption.ActionDelayed == DelayedAction.SPAWNING &&
                        other.SpawnDelayOption is NPCDelayTime delayOption &&
                        delayOption.DelayType == DelayKind.FROM_BEGINNING &&
                        delayOption.DelayAmount.Equals(NPCDelayTime.DUMMY_DELAY_AMOUNT));
                    if (others.Count == 0)
                        throw new InvalidScriptException(
                            "Cannot detect the leading NPC of the cut-out NPC for the cut-out scenario");
                    if (others.Count > 1)
                    {
                        string allName = "";
                        others.ForEach(oth => allName += oth.Name + ", ");
                        Debug.LogError("Found more than one NPCs for the leading NPC of the cut-out NPC: " 
                                       + allName + " use the first one by default.");
                    }

                    cutOutLaneChange.LeadVehicle = others[0];
                }
            }
        }

        private bool RetrieveEgo(FunctionExpContext egoFuncExp, ref Simulation simulation)
        {
            if (egoFuncExp.exception != null)
                throw new InvalidScriptException("Catch exception: " + egoFuncExp.exception.Message +
                    " in function " + egoFuncExp.GetText());
            FunctionScore func = new FunctionParser(egoFuncExp).Parse();
            if (func.Name != FunctionParser.FUNCTION_EGO)
                throw new InvalidScriptException("Expected Ego function but get: " + func.Name);
            if (func.Parameters.Count < 2)
                throw new InvalidScriptException("Ego function must have at least 2 arguments (initial position and goal): " +
                    ParserUtils.ToString(func.Parameters));

            // 1st arg: always initial position
            IPosition initPosition = ParsePosition(func.Parameters[0].children[0]);

            // 2nd arg: always goal
            IPosition goal = ParsePosition(func.Parameters[1].children[0]);
            var egoSettings = new EgoSettings(initPosition, goal);

            // 3rd arg (optional): setting options, e.g., max velocity
            if (func.Parameters.Count > 2)
            {
                ParseEgoConfig(func.Parameters[2].children[0], ref egoSettings);
            }
            
            simulation.Ego = egoSettings;
            return true;
        }
        
        private bool ParseEgoConfig(IParseTree node, ref EgoSettings egoSettings)
        {
            if (node is ArrayExpContext arrayExp)
            {
                List<ExpressionContext> expContexts = ParserUtils.ParseArray(arrayExp);
                foreach (var expContext in expContexts)
                    DoParseEgoConfig(expContext.children[0], ref egoSettings);
            }
            else if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseEgoConfig(scenarioScore.Variables[varName].children[0], ref egoSettings);
            }
            else
                throw new InvalidScriptException("Cannot parse config: " + node.GetText());
            return true;
        }
        
        private bool DoParseEgoConfig(IParseTree node, ref EgoSettings egoSettings)
        {
            if (node is EgoSettingExpContext egoSettingExp)
            {
                switch (egoSettingExp.children[0].GetText())
                {
                    case EGO_MAX_VELOCITY:
                        bool ok = ParseNumber(egoSettingExp.children[2], out float maxVel);
                        if (ok)
                            egoSettings.MaxVelocity = maxVel;
                        return true;
                    default:
                        throw new InvalidScriptException("Cannot parse the config: " + egoSettingExp.GetText());
                }
            }
            if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return DoParseEgoConfig(scenarioScore.Variables[varName].children[0], ref egoSettings);
            }
            throw new InvalidScriptException("Cannot parse the ego setting: " + node.GetText());
        }

        private bool RetrieveNPC(ExpressionContext npcExpContext, ref Simulation simulation)
        {
            if (npcExpContext.children[0] is VariableExpContext)
            {
                VariableExpContext varExp = (VariableExpContext)npcExpContext.children[0];
                if (!scenarioScore.Variables.ContainsKey(varExp.GetText()))
                    throw new InvalidScriptException("Undefined variable: " + varExp.GetText());
                ExpressionContext tempExp = scenarioScore.Variables[varExp.GetText()];
                if (tempExp.children == null ||
                    tempExp.children.Count == 0 ||
                    !(tempExp.children[0] is FunctionExpContext))
                    throw new InvalidScriptException("Expected a function defining NPC, but get: "
                        + tempExp.GetText());
                RetriveNPC((FunctionExpContext)tempExp.children[0], ref simulation, varExp.GetText());
            }
            else if (npcExpContext.children[0] is FunctionExpContext)
            {
                RetriveNPC((FunctionExpContext)npcExpContext.children[0], ref simulation);
            }
            return true;
        }

        private bool RetriveNPC(FunctionExpContext npcFuncContext, ref Simulation simulation, string npcName = "")
        {
            if (npcFuncContext.exception != null)
                throw new InvalidScriptException("Catch exception: " + npcFuncContext.exception.Message +
                    " in function " + npcFuncContext.GetText());
            FunctionScore func = new FunctionParser(npcFuncContext).Parse();
            if (func.Name != FunctionParser.FUNCTION_NPC)
                throw new InvalidScriptException("Expected NPC function but get: " + func.Name);

            if (func.Parameters.Count < 2)
                throw new InvalidScriptException("NPC function must have at least 2 arguments (vehicle type and position to be spawned): " +
                    ParserUtils.ToString(func.Parameters));

            // 1st vehicle type
            string vehicleTypeStr = ParseVehicleType(func.Parameters[0].children[0]);
            VehicleType vehicleType = ParseVehicleType(vehicleTypeStr);

            // 2nd arg: always spawn position
            IPosition spawnPosition = ParsePosition(func.Parameters[1].children[0]);

            List<Tuple<string, float>> route = new List<Tuple<string, float>>();
            IPosition goal = LaneOffsetPosition.DummyPosition();
            INPCSpawnDelay delay = NPCDelayTime.DummyDelay();
            NPCConfig config = NPCConfig.DummyConfigWithoutRoute();
            

            // 3rd arg (optional): goal or config option
            if (func.Parameters.Count >= 3)
            {
                switch (GetParamType(func.Parameters[2].children[0]))
                {
                    // goal
                    case ParamType.POSITION:
                        goal = ParsePosition(func.Parameters[2].children[0]);
                        break;
                    // config option
                    case ParamType.CONFIG:
                        ParseConfig(func.Parameters[2].children[0], ref delay, ref config);
                        break;
                    default:
                        throw new InvalidScriptException("The third argument of NPC function is invalid " +
                            "(it should be goal or delay config): " + func.Parameters[2].children[0].GetText());
                }
            }
            // 4th arg (optional): route (and speeds limit) or config option
            if (func.Parameters.Count >= 4)
            {
                switch (GetParamType(func.Parameters[3].children[0]))
                {
                    // route and speeds limit config
                    case ParamType.ROUTE_AND_SPEEDs_LIMIT:
                        route = ParseRouteAndSpeedsLimit(func.Parameters[3].children[0], 
                            out bool hasLaneChange, out ILaneChange laneChangeConfig);
                        if (hasLaneChange)
                        {
                            config.LaneChange = laneChangeConfig;
                        }
                        break;
                    // config option
                    case ParamType.CONFIG:
                        ParseConfig(func.Parameters[3].children[0], ref delay, ref config);
                        break;
                    default:
                        throw new InvalidScriptException("The fourth argument of NPC function is invalid " +
                            "(it should be route and speeds limit config or delay config): " + func.Parameters[3].children[0].GetText());
                }
            }

            // 5th arg (optional): config option
            if (func.Parameters.Count >= 5)
            {
                switch (GetParamType(func.Parameters[4].children[0]))
                {
                    // config option
                    case ParamType.CONFIG:
                        ParseConfig(func.Parameters[4].children[0], ref delay, ref config);
                        break;
                    default:
                        throw new InvalidScriptException("The fifth argument of NPC function is invalid " +
                            "(it should be delay config): " + func.Parameters[4].children[0].GetText());
                }
            }

            NPCCar npc = new NPCCar(vehicleType, spawnPosition);
            npc.Config = config;
            if (!(goal.Equals(LaneOffsetPosition.DummyPosition())))
                npc.Goal = goal;
            if (route.Count > 0)
                npc.Config.RouteAndSpeeds = route;
            if (!delay.Equals(NPCDelayTime.DummyDelay()))
                npc.SpawnDelayOption = delay;
            if (npcName != "")
                npc.Name = npcName;
            simulation.NPCs.Add(npc);
            return true;
        }

        private string ParseVehicleType(IParseTree node)
        {
            bool ok = ParseString(node, out string vehicleTypeStr);
            if (ok)
                return vehicleTypeStr;
            throw new InvalidScriptException("Cannot parse vehicle type from: " + node.GetText());
        }

        private static VehicleType ParseVehicleType(string vehicleType)
        {
            switch (vehicleType.ToLower())
            {
                case "taxi":
                    return VehicleType.TAXI;
                case "hatchback":
                    return VehicleType.HATCHBACK;
                case "small-car":
                case "smallcar":
                    return VehicleType.SMALL_CAR;
                case "truck":
                    return VehicleType.TRUCK;
                case "van":
                    return VehicleType.VAN;
                default:
                    throw new InvalidScriptException("Cannot parse the vehicle type: " + vehicleType);
            }
        }

        private IPosition ParsePosition(IParseTree node)
        {
            if (node is StringExpContext stringExp0)
            {
                string laneName = ParserUtils.ParseStringExp(stringExp0);
                return new LaneOffsetPosition(laneName);
            }
            if (node is PositionExpContext positionExp)
            {
                // absolute position - pair of traffic lane and offset
                if (positionExp.children[0] is StringExpContext stringExp)
                {
                    string laneName = ParserUtils.ParseStringExp(stringExp);
                    float offset = 0;
                    if (positionExp.ChildCount > 2)
                    {
                        bool ok = ParseNumber(positionExp.children[2], out float offset2);
                        if (ok) offset = offset2;
                    }
                    return new LaneOffsetPosition(laneName, offset);
                }
                // realtive position
                else
                {
                    IPosition reference = ParsePosition(positionExp.children[0]);
                    ParseNumber(positionExp.children[2], out float offset);
                    switch (positionExp.children[1].GetText())
                    {
                        case RELATIVE_POSITION_LEFT:
                            return new RelativePosition(reference, RelativePositionSide.LEFT, offset);
                        case RELATIVE_POSITION_RIGHT:
                            return new RelativePosition(reference, RelativePositionSide.RIGHT, offset);
                        case RELATIVE_POSITION_FORWARD:
                            return new RelativePosition(reference, RelativePositionSide.FORWARD, offset);
                        case RELATIVE_POSITION_BACK:
                            return new RelativePosition(reference, RelativePositionSide.FORWARD, -offset);
                    }
                }
            }
            else if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (varName == IGNORE_EXP)
                {
                    return LaneOffsetPosition.DummyPosition();
                }
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParsePosition(scenarioScore.Variables[varName].children[0]);
            }
            throw new InvalidScriptException("Cannot parse position from: " + node.GetText());
        }

        private List<Tuple<string, float>> ParseRouteAndSpeedsLimit(IParseTree node, 
            out bool hasLaneChange, out ILaneChange laneChangeConfig)
        {
            if (node is ArrayExpContext arrayExp)
            {
                hasLaneChange = false;
                laneChangeConfig = null;
                List<ExpressionContext> expContexts = ParserUtils.ParseArray(arrayExp);
                List<Tuple<string, float>> result = new List<Tuple<string, float>>();
                foreach (var expContext in expContexts)
                {
                    if (expContext.children[0] is RoadExpContext roadExp)
                    {
                        // a lane change
                        if (roadExp.children[0].GetText() == CHANGE_LANE)
                        {
                            hasLaneChange = true;
                            laneChangeConfig = new LaneChangeConfig();
                            laneChangeConfig.SourceLane = result.Last().Item1;
                            var arguments =
                                ParserUtils.ParseFuncArgs((ArgumentListContext)roadExp.children[2]);
                            ParseLaneChange(arguments, ref laneChangeConfig);
                        }
                        // cut in
                        else if (roadExp.children[0].GetText() == CUT_IN)
                        {
                            hasLaneChange = true;
                            laneChangeConfig = new CutInLaneChange();
                            laneChangeConfig.SourceLane = result.Last().Item1;
                            var arguments =
                                ParserUtils.ParseFuncArgs((ArgumentListContext)roadExp.children[2]);
                            ParseLaneChange(arguments, ref laneChangeConfig);
                            if (arguments.Count < 4)
                            {
                                throw new InvalidScriptException("Cut in scenario requires at least 4 arguments. " +
                                                                 "The 4th argument specifies dx");
                            }

                            bool ok = ParseNumber(arguments[3], out float dx);
                            (laneChangeConfig as CutInLaneChange).Dx = dx;
                        }
                        // cut out
                        else if (roadExp.children[0].GetText() == CUT_OUT)
                        {
                            hasLaneChange = true;
                            laneChangeConfig = new CutOutLaneChange();
                            laneChangeConfig.SourceLane = result.Last().Item1;
                            var arguments =
                                ParserUtils.ParseFuncArgs((ArgumentListContext)roadExp.children[2]);
                            ParseLaneChange(arguments, ref laneChangeConfig);
                            if (arguments.Count < 4)
                            {
                                throw new InvalidScriptException("Cut out scenario requires at least 4 arguments. " +
                                                                 "The 4th argument specifies dx_f");
                            }

                            bool ok = ParseNumber(arguments[3], out float dx_f);
                            (laneChangeConfig as CutOutLaneChange).Dxf = dx_f;
                        }
                    }
                    // pair of traffic lane and desired speed limit
                    if (expContext.children[0] is StringExpContext ||
                        (expContext.children[0] is RoadExpContext roadExp2 &&
                         roadExp2.children[0] is StringExpContext))
                    {
                        ParseRoute(expContext.children[0], ref result);
                        if (laneChangeConfig?.TargetLane == null &&
                            laneChangeConfig?.SourceLane != null &&
                            laneChangeConfig?.SourceLane == result[^2].Item1)
                        {
                            laneChangeConfig.TargetLane = result.Last().Item1;
                            if (laneChangeConfig.LongitudinalVelocity == 0)
                            {
                                string sourceLane = laneChangeConfig.SourceLane;
                                laneChangeConfig.LongitudinalVelocity =
                                    result.Find(tuple => tuple.Item1 == sourceLane).Item2;
                            }
                        }
                    }
                
                }
                return result;
            }
            if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseRouteAndSpeedsLimit(scenarioScore.Variables[varName].children[0], 
                    out hasLaneChange, out laneChangeConfig);
            }
            throw new InvalidScriptException("Cannot parse route and speeds limit from: " +
                node.GetText());
        }

        private bool ParseRoute(IParseTree node, ref List<Tuple<string, float>> result)
        {
            if (node is StringExpContext)
            {
                result.Add(new Tuple<string, float>(
                    ParserUtils.ParseStringExp((StringExpContext)node), NPCConfig.DUMMY_SPEED));
                return true;
            }
            if (node is RoadExpContext roadExp)
            {
                if (roadExp.children[0] is StringExpContext stringExp)
                {
                    string laneName = ParserUtils.ParseStringExp(stringExp);
                    float speed = NPCConfig.DUMMY_SPEED;
                    if (roadExp.ChildCount > 3)
                    {
                        bool ok = ParseNumber(roadExp.children[3], out float speed2);
                        if (ok) speed = speed2;
                    }

                    result.Add(new Tuple<string, float>(laneName, speed));
                }
                return true;
            }
            throw new InvalidScriptException("Cannot parse route and speed from: " +
                node.GetText());
        }

        /// <returns>false if the expression is variable "_"</returns>
        private bool ParseNumber(IParseTree node, out float result)
        {
            if (node is NumberExpContext numberExp)
            {
                result = ParserUtils.ParseNumberExp(numberExp);
                return true;
            }

            if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (varName == IGNORE_EXP)
                {
                    result = 0;
                    return false;
                }
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseNumber(scenarioScore.Variables[varName], out result);
            }

            if (node is ExpressionContext expContext)
            {
                return ParseNumber(expContext.children[0], out result);
            }
            throw new InvalidScriptException($"Expected a number, but get: {node.GetText()}");
        }

        private bool ParseString(IParseTree node, out string result)
        {
            if (node is StringExpContext stringExp)
            {
                result = ParserUtils.ParseStringExp(stringExp);
                return true;
            }

            if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (varName == IGNORE_EXP)
                {
                    result = "";
                    return false;
                }
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseString(scenarioScore.Variables[varName], out result);
            }

            if (node is ExpressionContext expContext)
            {
                return ParseString(expContext.children[0], out result);
            }
            throw new InvalidScriptException($"Expected a string, but get: {node.GetText()}");
        }

        private void ParseLaneChange(List<ExpressionContext> argExpContexts, ref ILaneChange laneChangeConfig)
        {
            if (argExpContexts.Count < 3)
            {
                throw new InvalidScriptException("Change lane expression requires at least 3 arguments " +
                                                 "including offset position, longitudinal velocity and lateral velocity.");
            }
            bool ok = ParseNumber(argExpContexts[0], out float offset);
            if (!ok)
                offset = ILaneChange.DUMMY_CHANGE_OFFSET;
            laneChangeConfig.ChangeOffset = offset;
            
            ok = ParseNumber(argExpContexts[1], out float longVel);
            laneChangeConfig.LongitudinalVelocity = longVel;
            
            ok = ParseNumber(argExpContexts[2], out float latVel);
            if (!ok)
                latVel = ILaneChange.DEFAULT_LATERAL_VELOCITY;
            laneChangeConfig.LateralVelocity = latVel;
        }

        private bool ParseConfig(IParseTree node, ref INPCSpawnDelay spawnDelay, ref NPCConfig npcConfig)
        {
            if (node is ArrayExpContext arrayExp)
            {
                List<ExpressionContext> expContexts = ParserUtils.ParseArray(arrayExp);
                foreach (var expContext in expContexts)
                    DoParseConfig(expContext.children[0], ref spawnDelay, ref npcConfig);
            }
            else if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseConfig(scenarioScore.Variables[varName].children[0], ref spawnDelay, ref npcConfig);
            }
            else
                throw new InvalidScriptException("Cannot parse config: " + node.GetText());
            return true;
        }
        
        // return 0 if spawnDelay is updated, 1 if npcConfig is updated
        private int DoParseConfig(IParseTree node, ref INPCSpawnDelay spawnDelay, ref NPCConfig npcConfig)
        {
            if (node is ConfigExpContext configExp)
            {
                switch (configExp.children[0].GetText())
                {
                    case AGGRESSIVE_DRIVING:
                        npcConfig.AggresiveDrive = true;
                        return 1;
                    case ACCELERATION:
                        bool ok = ParseNumber(configExp.children[2], out float accel);
                        if (ok) npcConfig.Acceleration = accel;
                        return 1;
                    case DECELERATION:
                        bool ok2 = ParseNumber(configExp.children[2], out float decel);
                        if (ok2) npcConfig.Deceleration = decel;
                        return 1;
                    case DELAY_SPAWN:
                    case DELAY_MOVE:
                    case DELAY_SPAWN_UNTIL_EGO_ENGAGED:
                    case DELAY_MOVE_UNTIL_EGO_ENGAGED:
                    case DELAY_SPAWN_UNTIL_EGO_MOVE:
                    case DELAY_MOVE_UNTIL_EGO_MOVE:
                        spawnDelay = ParseDelayOption(configExp);
                        return 0;
                    default:
                        throw new InvalidScriptException("Cannot parse the config: " + configExp.GetText());

                }
            }
            if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return DoParseConfig(scenarioScore.Variables[varName].children[0], ref spawnDelay, ref npcConfig);
            }
            throw new InvalidScriptException("Cannot parse the config: " + node.GetText());
        }

        private NPCDelayTime ParseDelayOption(ConfigExpContext configExp)
        {
            bool ok = ParseNumber(configExp.children[2], out float amount);
            // case "_"
            if (!ok)
                amount = NPCDelayTime.DUMMY_DELAY_AMOUNT;
            switch (configExp.children[0].GetText())
            {
                case DELAY_SPAWN:
                    return NPCDelayTime.DelaySpawn(amount);
                case DELAY_MOVE:
                    return NPCDelayTime.DelayMove(amount);
                case DELAY_SPAWN_UNTIL_EGO_ENGAGED:
                    return NPCDelayTime.DelaySpawnUntilEgoEngaged(amount);
                case DELAY_MOVE_UNTIL_EGO_ENGAGED:
                    return NPCDelayTime.DelayMoveUntilEgoEngaged(amount);
                case DELAY_SPAWN_UNTIL_EGO_MOVE:
                    return NPCDelayTime.DelaySpawnUntilEgoMove(amount);
                case DELAY_MOVE_UNTIL_EGO_MOVE:
                    return NPCDelayTime.DelayMoveUntilEgoMove(amount);
                default:
                    throw new InvalidScriptException("Cannot parse " + configExp.GetText());
            }
        }

        private bool ParseSimulationConfig(IParseTree node, ref Simulation simulation)
        {
            if (node is ArrayExpContext arrayExpContext)
            {
                var simulationConfigs = ParserUtils.ParseArray(arrayExpContext);
                foreach (var simulationConfig in simulationConfigs)
                {
                    if (simulationConfig.children[0] is SimulationSettingExpContext simulationSettingExp)
                    {
                        ParseNumber(simulationSettingExp.children[2], out float timeout);
                        simulation.SavingTimeout = timeout;
                    }
                    else throw new InvalidScriptException("Unexpected argument: " + simulationConfig.GetText());
                }
                return true;
            }
            throw new InvalidScriptException("Expected an array of settings for simulation, but get: " + node.GetText());
        }

        // if node is a param of position
        private ParamType GetParamType(IParseTree node)
        {
            if (node is StringExpContext or PositionExpContext)
                return ParamType.POSITION;
            if (node is ArrayExpContext arrayExp)
            {
                List<ExpressionContext> expContexts = ParserUtils.ParseArray(arrayExp);
                if (expContexts.Count == 0)
                    return ParamType.ROUTE_AND_SPEEDs_LIMIT;
                if (IsAnArrayOfRoad(expContexts))
                    return ParamType.ROUTE_AND_SPEEDs_LIMIT;
                return ParamType.CONFIG;
            }
            if (node is VariableExpContext)
            {
                string varName = ((VariableExpContext)node).children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return GetParamType(scenarioScore.Variables[varName].children[0]);
            }
            throw new InvalidScriptException("Cannot recognize param type: " +
                node.GetText());
        }

        // check if the given argument forms a route
        private bool IsAnArrayOfRoad(List<ExpressionContext> expContexts)
        {
            foreach (var expContext in expContexts)
            {
                if (!(expContext.children[0] is StringExpContext ||
                      expContext.children[0] is RoadExpContext))
                    return false;
            }
            return true;
        }
    }
    public enum ParamType
    {
        POSITION,
        ROUTE_AND_SPEEDs_LIMIT,
        CONFIG,
    }
}

