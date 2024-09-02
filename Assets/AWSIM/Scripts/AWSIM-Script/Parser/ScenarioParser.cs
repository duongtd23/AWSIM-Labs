using UnityEngine;
using Antlr4.Runtime.Tree;
using AWSIM_Script.Error;
using AWSIM_Script.Object;
using AWSIM_Script.Parser.Object;
using static AWSIMScriptGrammarParser;
using System.Collections.Generic;
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
        
        private ScenarioScore scenarioScore;
        public ScenarioParser(ScenarioScore scenarioScore)
		{
            this.scenarioScore = scenarioScore;
		}

        public Scenario Parse()
        {
            var runFuncs = scenarioScore.Functions.FindAll(function => function.Name == FunctionParser.FUNCTION_RUN);
            if (runFuncs.Count == 0)
            {
                Debug.LogWarning("[AWAnalysis] There is no run function in the input script.");
                return new Scenario();
            }
            if (runFuncs.Count > 1)
                Debug.LogError("[AWAnalysis] Found more than run function in the input script." +
                    " Use the last one by default.");
            var runFunc = runFuncs[^1];

            if (runFunc.Parameters.Count < 1)
                throw new InvalidScriptException("Invalid arguments passed for function run: ");

            Scenario scenario = new Scenario();
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
            if (runFunc.Parameters.Count > 1 &&
                runFunc.Parameters[1].children?[0] is ArrayExpContext)
            {
                npcsExpContexts = ParserUtils.
                ParseArray((ArrayExpContext)runFunc.Parameters[1].children[0]);
            }
            else
                throw new InvalidScriptException("Invalid arguments passed for function run: ");

            // Prase Ego
            if (egoFuncExp != FunctionExpContext.EmptyContext)
                RetrieveEgo((FunctionExpContext)egoFuncExp, ref scenario);

            // Parse NPCs
            foreach (ExpressionContext npcExpContext in npcsExpContexts)
            {
                RetrieveNPC(npcExpContext, ref scenario);
            }
            return scenario;
        }

        private bool RetrieveEgo(FunctionExpContext egoFuncExp, ref Scenario scenario)
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
            
            scenario.Ego = egoSettings;
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
                        egoSettings.MaxVelocity = ParserUtils.ParseNumberExp((NumberExpContext)egoSettingExp.children[2]);
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

        private bool RetrieveNPC(ExpressionContext npcExpContext, ref Scenario scenario)
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
                RetriveNPC((FunctionExpContext)tempExp.children[0], ref scenario, varExp.GetText());
            }
            else if (npcExpContext.children[0] is FunctionExpContext)
            {
                RetriveNPC((FunctionExpContext)npcExpContext.children[0], ref scenario);
            }
            return true;
        }

        private bool RetriveNPC(FunctionExpContext npcFuncContext, ref Scenario scenario, string npcName = "")
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

            Dictionary<string, float> route = new Dictionary<string, float>();
            IPosition goal = LaneOffsetPosition.DummyPosition();
            NPCSpawnDelay delay = NPCSpawnDelay.DummyDelay();
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
                        route = ParseRouteAndSpeedsLimit(func.Parameters[3].children[0]);
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
            if (!(delay.Equals(NPCSpawnDelay.DummyDelay())))
                npc.SpawnDelayOption = delay;
            if (npcName != "")
                npc.Name = npcName;
            scenario.NPCs.Add(npc);
            return true;
        }

        private string ParseVehicleType(IParseTree node)
        {
            if (node is StringExpContext)
            {
                return ParserUtils.ParseStringExp((StringExpContext)node);
            }
            else if (node is VariableExpContext)
            {
                string varName = ((VariableExpContext)node).children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseVehicleType(scenarioScore.Variables[varName].children[0]);
            }
            else
            {
                throw new InvalidScriptException("Cannot parse vehicle type from: " +
                    node.GetText());
            }
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
            else if (node is PositionExpContext positionExp)
            {
                // absolute position - pair of traffic lane and offset
                if (positionExp.children[0] is StringExpContext stringExp)
                {
                    string laneName = ParserUtils.ParseStringExp(stringExp);
                    float offset = 0;
                    if (positionExp.ChildCount > 2)
                        offset = ParserUtils.ParseNumberExp((NumberExpContext)positionExp.children[2]);
                    return new LaneOffsetPosition(laneName, offset);
                }
                // realtive position
                else
                {
                    IPosition reference = ParsePosition(positionExp.children[0]);
                    float offset = ParserUtils.ParseNumberExp((NumberExpContext)positionExp.children[2]);
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
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParsePosition(scenarioScore.Variables[varName].children[0]);
            }
            throw new InvalidScriptException("Cannot parse position from: " + node.GetText());
        }

        private Dictionary<string, float> ParseRouteAndSpeedsLimit(IParseTree node)
        {
            if (node is ArrayExpContext arrayExp)
            {
                List<ExpressionContext> expContexts = ParserUtils.ParseArray(arrayExp);
                Dictionary<string, float> result = new Dictionary<string, float>();
                foreach (var expContext in expContexts)
                    ParseRoute(expContext.children[0], ref result);
                return result;
            }
            else if (node is VariableExpContext variableExp)
            {
                string varName = variableExp.children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseRouteAndSpeedsLimit(scenarioScore.Variables[varName].children[0]);
            }
            else
            {
                throw new InvalidScriptException("Cannot parse route and speeds limit from: " +
                    node.GetText());
            }
        }

        private bool ParseRoute(IParseTree node, ref Dictionary<string, float> result)
        {
            if (node is StringExpContext)
            {
                result.Add(ParserUtils.ParseStringExp((StringExpContext)node), NPCConfig.DUMMY_SPEED);
                return true;
            }
            else if (node is RoadExpContext roadExp)
            {
                string laneName = ParserUtils.ParseStringExp((StringExpContext)(roadExp.children[0]));
                float speed = NPCConfig.DUMMY_SPEED;
                if (roadExp.ChildCount > 2)
                    speed = ParserUtils.ParseNumberExp((NumberExpContext)roadExp.children[2]);
                result.Add(laneName, speed);
                return true;
            }
            else
            {
                throw new InvalidScriptException("Cannot parse route and speed from: " +
                    node.GetText());
            }
        }

        private bool ParseConfig(IParseTree node, ref NPCSpawnDelay spawnDelay, ref NPCConfig npcConfig)
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
        private int DoParseConfig(IParseTree node, ref NPCSpawnDelay spawnDelay, ref NPCConfig npcConfig)
        {
            if (node is ConfigExpContext configExp)
            {
                switch (configExp.children[0].GetText())
                {
                    case AGGRESSIVE_DRIVING:
                        npcConfig.AggresiveDrive = true;
                        return 1;
                    case ACCELERATION:
                        npcConfig.Acceleration = ParserUtils.ParseNumberExp((NumberExpContext)configExp.children[2]);
                        return 1;
                    case DECELERATION:
                        npcConfig.Deceleration = ParserUtils.ParseNumberExp((NumberExpContext)configExp.children[2]);
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

        private NPCSpawnDelay ParseDelayOption(ConfigExpContext configExp)
        {
            float amount = ParserUtils.ParseNumberExp((NumberExpContext)configExp.children[2]);
            switch (configExp.children[0].GetText())
            {
                case DELAY_SPAWN:
                    return NPCSpawnDelay.DelaySpawn(amount);
                case DELAY_MOVE:
                    return NPCSpawnDelay.DelayMove(amount);
                case DELAY_SPAWN_UNTIL_EGO_ENGAGED:
                    return NPCSpawnDelay.DelaySpawnUntilEgoEngaged(amount);
                case DELAY_MOVE_UNTIL_EGO_ENGAGED:
                    return NPCSpawnDelay.DelayMoveUntilEgoEngaged(amount);
                case DELAY_SPAWN_UNTIL_EGO_MOVE:
                    return NPCSpawnDelay.DelaySpawnUntilEgoMove(amount);
                case DELAY_MOVE_UNTIL_EGO_MOVE:
                    return NPCSpawnDelay.DelayMoveUntilEgoMove(amount);
                default:
                    throw new InvalidScriptException("Cannot parse " + configExp.GetText());
            }
        }

        // if node is a param of position
        private ParamType GetParamType(IParseTree node)
        {
            if (node is StringExpContext || node is PositionExpContext)
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

        // check if the given argument form a route
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

