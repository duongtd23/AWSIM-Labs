using UnityEngine;
using Antlr4.Runtime.Tree;
using AWSIM_Script.Error;
using AWSIM_Script.Object;
using AWSIM_Script.Parser.Object;
using static AWSIMScriptGrammarParser;
using System.Collections.Generic;

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

        private ScenarioScore scenarioScore;
        public ScenarioParser(ScenarioScore scenarioScore)
		{
            this.scenarioScore = scenarioScore;
		}

        public Scenario Parse()
        {
            var runFuncs = scenarioScore.Functions.FindAll(function => function.Name == FunctionParser.FUNCTION_RUN);
            if (runFuncs.Count == 0)
                throw new InvalidScriptException("There is no run function in the input script.");
            if (runFuncs.Count > 1)
                Debug.LogError("[ERROR] Found more than run function in the input script." +
                    " Use the last one by default.");
            var runFunc = runFuncs[runFuncs.Count - 1];

            if (runFunc.Parameters.Count < 1)
                throw new InvalidScriptException("Invalid arguments passed for function run: ");
            if (runFunc.Parameters[0].children == null ||
                runFunc.Parameters[0].children[0] == null ||
                !(runFunc.Parameters[0].children[0] is ArrayExpContext))
                throw new InvalidScriptException("Invalid arguments passed for function run: ");

            List<ExpressionContext> npcsExpContexts = ParserUtils.
                ParseArray((ArrayExpContext)runFunc.Parameters[0].children[0]);

            Scenario scenario = new Scenario();
            foreach (ExpressionContext npcExpContext in npcsExpContexts)
            {
                RetriveNPC(npcExpContext, ref scenario);
            }
            return scenario;
        }

        private bool RetriveNPC(ExpressionContext npcExpContext, ref Scenario scenario)
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

            // 3rd arg (optional): goal or delay option
            if (func.Parameters.Count >= 3)
            {
                switch (GetParamType(func.Parameters[2].children[0]))
                {
                    // goal
                    case ParamType.POSITION:
                        goal = ParsePosition(func.Parameters[2].children[0]);
                        break;
                    // delay option
                    case ParamType.DELAY_OPTION:
                        delay = ParseDelayOption(func.Parameters[2].children[0]);
                        break;
                    default:
                        throw new InvalidScriptException("The third argument of NPC function is invalid " +
                            "(it should be goal or delay config): " + func.Parameters[2].children[0].GetText());
                }
            }
            // 4th arg (optional): route (and speeds limit) or delay option
            if (func.Parameters.Count >= 4)
            {
                switch (GetParamType(func.Parameters[3].children[0]))
                {
                    // route and speeds limit config
                    case ParamType.ROUTE_AND_SPEEDs_LIMIT:
                        route = ParseRouteAndSpeedsLimit(func.Parameters[3].children[0]);
                        break;
                    // delay option
                    case ParamType.DELAY_OPTION:
                        delay = ParseDelayOption(func.Parameters[3].children[0]);
                        break;
                    default:
                        throw new InvalidScriptException("The fourth argument of NPC function is invalid " +
                            "(it should be route and speeds limit config or delay config): " + func.Parameters[3].children[0].GetText());
                }
            }

            // 5th arg (optional): delay option
            if (func.Parameters.Count >= 5)
            {
                switch (GetParamType(func.Parameters[4].children[0]))
                {
                    // delay option
                    case ParamType.DELAY_OPTION:
                        delay = ParseDelayOption(func.Parameters[4].children[0]);
                        break;
                    default:
                        throw new InvalidScriptException("The fifth argument of NPC function is invalid " +
                            "(it should be delay config): " + func.Parameters[4].children[0].GetText());
                }
            }

            NPCCar npc = new NPCCar(vehicleType, spawnPosition);
            if (!(goal.Equals(LaneOffsetPosition.DummyPosition())))
                npc.Goal = goal;
            if (route.Count > 0)
                npc.Config = new NPCConfig(route);
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
            if (node is StringExpContext)
            {
                string laneName = ParserUtils.ParseStringExp((StringExpContext)node);
                return new LaneOffsetPosition(laneName);
            }
            else if (node is PositionExpContext)
            {
                if (((PositionExpContext)node).children[0] is LanePositionExpContext)
                {
                    LanePositionExpContext lanePositionExpContext = (LanePositionExpContext)((PositionExpContext)node).children[0];
                    string laneName = ParserUtils.ParseStringExp((StringExpContext)lanePositionExpContext.children[0]);
                    float offset = 0;
                    if (lanePositionExpContext.ChildCount > 2)
                        offset = ParserUtils.ParseNumberExp((NumberExpContext)lanePositionExpContext.children[2]);
                    return new LaneOffsetPosition(laneName, offset);
                }
                // parsing other types of position (e.g., relative position) should be done here
                else
                    throw new InvalidScriptException("Cannot parse position from: " +
                    node.GetText());
            }
            else if (node is VariableExpContext)
            {
                string varName = ((VariableExpContext)node).children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParsePosition(scenarioScore.Variables[varName].children[0]);
            }
            else
            {
                throw new InvalidScriptException("Cannot parse position from: " +
                    node.GetText());
            }
        }

        private Dictionary<string, float> ParseRouteAndSpeedsLimit(IParseTree node)
        {
            if (node is ArrayExpContext)
            {
                List<ExpressionContext> expContexts =
                    ParserUtils.ParseArray((ArrayExpContext)node);
                Dictionary<string, float> result = new Dictionary<string, float>();
                foreach (var expContext in expContexts)
                    ParseRoute(expContext.children[0], ref result);
                return result;
            }
            else if (node is VariableExpContext)
            {
                string varName = ((VariableExpContext)node).children[0].GetText();
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
            else if (node is RouteExpContext)
            {
                RouteExpContext routeExp = (RouteExpContext)node;
                string laneName = ParserUtils.ParseStringExp((StringExpContext)(routeExp.children[0]));
                float speed = NPCConfig.DUMMY_SPEED;
                if (routeExp.ChildCount > 2)
                    speed = ParserUtils.ParseNumberExp((NumberExpContext)routeExp.children[2]);
                result.Add(laneName, speed);
                return true;
            }
            else
            {
                throw new InvalidScriptException("Cannot parse route and speed from: " +
                    node.GetText());
            }
        }

        private NPCSpawnDelay ParseDelayOption(IParseTree node)
        {
            if (node is SpawnDelayOptionExpContext)
            {
                var delayExpContext = (SpawnDelayOptionExpContext)node;
                float amount = ParserUtils.ParseNumberExp((NumberExpContext)delayExpContext.children[2]);
                switch (delayExpContext.children[0].GetText())
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
                }
                throw new InvalidScriptException("");
            }
            else if (node is VariableExpContext)
            {
                string varName = ((VariableExpContext)node).children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return ParseDelayOption(scenarioScore.Variables[varName].children[0]);
            }
            else
            {
                throw new InvalidScriptException("Cannot parse delay option from: " +
                    node.GetText());
            }
        }

        // if node is a param of position
        private ParamType GetParamType(IParseTree node)
        {
            if (node is StringExpContext || node is PositionExpContext)
                return ParamType.POSITION;
            else if (node is SpawnDelayOptionExpContext)
                return ParamType.DELAY_OPTION;
            else if (node is ArrayExpContext)
                return ParamType.ROUTE_AND_SPEEDs_LIMIT;
            else if (node is VariableExpContext)
            {
                string varName = ((VariableExpContext)node).children[0].GetText();
                if (!scenarioScore.Variables.ContainsKey(varName))
                    throw new InvalidScriptException("Undefined variable: " + varName);
                return GetParamType(scenarioScore.Variables[varName].children[0]);
            }
            else
                throw new InvalidScriptException("Cannot recognize param type: " +
                    node.GetText());
        }

    }
    public enum ParamType
    {
        POSITION,
        ROUTE_AND_SPEEDs_LIMIT,
        DELAY_OPTION,
    }
}

