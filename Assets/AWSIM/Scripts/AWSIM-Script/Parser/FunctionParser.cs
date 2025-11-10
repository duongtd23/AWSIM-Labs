using Antlr4.Runtime.Tree;
using AWSIM_Script.Error;
using AWSIM_Script.Parser.Object;
using static AWSIMScriptGrammarParser;

namespace AWSIM_Script.Parser
{
    // parser for a function,
	public class FunctionParser
	{
        public const string FUNCTION_RUN = "run";
        public const string FUNCTION_NPC = "NPC";
        public const string FUNCTION_EGO = "Ego";
        public const string FUNCTION_PEDESTRIAN = "Pedes";

        private FunctionExpContext functionExpContext;
        public FunctionParser(FunctionExpContext functionExp)
        {
            this.functionExpContext = functionExp;
        }

        public FunctionScore Parse()
        {
            if (functionExpContext.children == null)
                throw new InvalidScriptException("Cannot parse the function: " + functionExpContext.GetText());
            if (functionExpContext.exception != null)
                throw new InvalidScriptException("Cannot parse the function: " + functionExpContext.GetText() +
                    ". Exception: " + functionExpContext.exception.Message);

            IdExpContext idExpContext = (IdExpContext)functionExpContext.children[0];

            FunctionScore func =
                new FunctionScore(idExpContext.children[0].GetText());

            // if function has argument(s)
            // functionExpContext.children[1] is the open parent thesis
            if (!(functionExpContext.children[2] is TerminalNodeImpl))
            {
                IParseTree argsNode = functionExpContext.children[2];
                func.Parameters =
                    ParserUtils.ParseFuncArgs((ArgumentListContext)argsNode);
            }
            return func;
        }
    }
}

