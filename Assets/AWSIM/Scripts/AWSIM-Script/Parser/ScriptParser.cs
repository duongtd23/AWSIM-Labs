using System;
using Antlr4.Runtime;
using Antlr4.Runtime.Tree;
using AWSIM_Script.Object;
using AWSIM_Script.Error;
using AWSIM_Script.Parser.Object;
using static AWSIMScriptGrammarParser;
using System.IO;

namespace AWSIM_Script.Parser
{
	public class ScriptParser
	{
		public ScriptParser()
		{
		}
        public Simulation ParseScriptFromFile(string scriptFilePath)
        {
            string scriptContent = File.ReadAllText(scriptFilePath);
            return ParseScript(scriptContent);
        }

        public Simulation ParseScript(string scriptContent)
        {
            ICharStream stream = CharStreams.fromString(scriptContent);
            ITokenSource lexer = new AWSIMScriptGrammarLexer(stream);
            ITokenStream tokens = new CommonTokenStream(lexer);
            AWSIMScriptGrammarParser parser = new AWSIMScriptGrammarParser(tokens);
            IParseTree tree = parser.scenario();
            return ParseScript(tree);
        }

        /// <summary>
        /// provide the prased tree, convert it to a Scenario object
        /// </summary>
        /// <param name="antlrTree"></param>
        /// <returns></returns>
        public Simulation ParseScript(IParseTree antlrTree)
        {
            if (!(antlrTree is ScenarioContext))
                throw new InvalidScriptException("Cannot parse the input to a Scenario object.");
            ScenarioContext scenarioContext = (ScenarioContext)antlrTree;
            if (scenarioContext.exception != null)
                throw new InvalidScriptException("Cannot parse the input. Exception: " + scenarioContext.exception.Message);

            ScenarioScore scenarioEnv = new ScenarioScore();
            foreach (IParseTree child in scenarioContext.children)
            {
                // EOF
                if (ParserUtils.IsEOFNode(child))
                    break;
                if (!(child is StatementContext))
                    throw new InvalidScriptException("Cannot parse the statement: " + child.GetText());
                StatementContext statementContext = (StatementContext)child;
                if (statementContext.exception != null)
                    throw new InvalidScriptException("Cannot parse statement, exception: " + statementContext.exception.Message);
                ParseStatementExp(statementContext, ref scenarioEnv);
            }
            return new ScenarioParser(scenarioEnv).Parse();
        }

        // the statementContext is already validated
        public bool ParseStatementExp(StatementContext statementContext,
            ref ScenarioScore scenarioEnv)
        {
            if (statementContext.children == null || statementContext.children[0] == null)
                throw new InvalidScriptException("Cannot parse the statement: " + statementContext.GetText());

            // validate semi colon exist at the end
            ParserUtils.ValidateSemiColon(statementContext.children[1]);

            // if this is a function
            if (statementContext.children[0] is FunctionExpContext)
            {
                FunctionExpContext functionContext = (FunctionExpContext)statementContext.children[0];
                if (functionContext.exception != null)
                    throw new InvalidScriptException("Cannot parse function, exception: " + functionContext.exception.Message);
                
                ParseFunction(functionContext, ref scenarioEnv);
            }

            // if this is an assignment statement
            else if (statementContext.children[0] is AssignmentStmContext)
            {
                AssignmentStmContext assignmentStmContext = (AssignmentStmContext)statementContext.children[0];
                if (assignmentStmContext.exception != null)
                    throw new InvalidScriptException("Cannot parse assignment statement, exception: " + assignmentStmContext.exception.Message);
                ParseAssignmentStm(assignmentStmContext, ref scenarioEnv);
            } else
                throw new InvalidScriptException("Cannot parse the statement: " + statementContext.GetText() + ". " +
                    "It should be a function or an assignment statement.");

            return true;
        }

        public bool ParseFunction(FunctionExpContext functionContext, ref ScenarioScore scenarioEnv)
        {
            FunctionScore func = new FunctionParser(functionContext).Parse();
            scenarioEnv.Functions.Add(func);
            return true;
        }

        public bool ParseAssignmentStm(AssignmentStmContext assignmentStmContext, ref ScenarioScore scenarioEnv)
        {
            new AssignStatementParser().Parse(assignmentStmContext, ref scenarioEnv);
            return true;
        }
    }
}

