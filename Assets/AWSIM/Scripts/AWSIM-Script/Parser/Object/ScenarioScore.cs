using static AWSIMScriptGrammarParser;
using System.Collections.Generic;

namespace AWSIM_Script.Parser.Object
{
	// intermediate class of scenario
	// store a list of functions and a map from variable names to expressions 
	public class ScenarioScore
	{
		public ScenarioScore(Dictionary<string, ExpressionContext> variables,
			List<FunctionScore> functions)
		{
			Variables = variables;
			Functions = functions;
		}
		public ScenarioScore()
		{
			Variables = new Dictionary<string, ExpressionContext>();
			Functions = new List<FunctionScore>();
		}

		// map from variables to their expression
		public Dictionary<string, ExpressionContext> Variables { get; set; }
		// all function
		public List<FunctionScore> Functions { get; set; }

	}
}

