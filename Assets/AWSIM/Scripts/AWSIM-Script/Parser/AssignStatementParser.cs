using UnityEngine;
using AWSIM_Script.Error;
using AWSIM_Script.Parser.Object;
using static AWSIMScriptGrammarParser;

namespace AWSIM_Script.Parser
{
	public class AssignStatementParser
	{
		public bool Parse(AssignmentStmContext assignmentStmContext, ref ScenarioScore scenarioEnv)
		{
			if (!(assignmentStmContext.children[0] is VariableExpContext))
				throw new InvalidScriptException("Cannot parse variable expression: " +
					assignmentStmContext.children[0].GetText());
            if (!(assignmentStmContext.children[2] is ExpressionContext))
                throw new InvalidScriptException("Cannot parse expression: " +
                    assignmentStmContext.children[2].GetText());

			string varName = assignmentStmContext.children[0].GetText();
			if (scenarioEnv.Variables.ContainsKey(varName))
				Debug.LogError("[ERROR]: Redefine the variable " + varName);
            scenarioEnv.Variables.Add(varName,
				(ExpressionContext)assignmentStmContext.children[2]);
			return true;
        }
	}
}

