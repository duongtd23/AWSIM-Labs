using System;
using System.Collections.Generic;
using static AWSIMScriptGrammarParser;

namespace AWSIM_Script.Parser.Object
{
    // intermediate function class
    // store the function name and its arguments
	public class FunctionScore
	{
        public string Name { get; set; }
        public List<ExpressionContext> Parameters { get; set; }

        public FunctionScore(string name, List<ExpressionContext> parameters)
        {
            Name = name;
            Parameters = parameters;
        }
        public FunctionScore(string name)
        {
            Name = name;
            Parameters = new List<ExpressionContext>();
        }
    }
}

