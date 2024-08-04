using System;
using UnityEngine;
using Antlr4.Runtime.Tree;
using AWSIM_Script.Error;
using static AWSIMScriptGrammarParser;
using System.Collections.Generic;

namespace AWSIM_Script.Parser
{
    public static class ParserUtils
    {
        public static string COLON = ",";
        public static string SEMI_COLON = ";";

        // validate that the node is a semi colon
        public static void ValidateSemiColon(IParseTree node)
        {
            if (node == null || !(node is TerminalNodeImpl))
                Debug.LogError("[ERROR] Look like you miss/wrongly declare a semi colon");
            if (node.GetText() != SEMI_COLON)
                Debug.LogError("[ERROR] A semi colon is expected at the end of: " + node.Parent.GetText());
        }

        // validate that the node is a colon
        public static void ValidateAColon(IParseTree node)
        {
            if (!(node is TerminalNodeImpl || node.GetText() == COLON))
                throw new InvalidScriptException("A semi colon is expected but get: " + node.GetText());
        }

        // check if the node is end-of-file
        public static bool IsEOFNode(IParseTree node)
        {
            return (node is TerminalNodeImpl && node.ChildCount == 0);
        }

        // override ToString of ExpressionContext list
        public static string ToString(List<ExpressionContext> expressionContexts)
        {
            string result = "";
            expressionContexts.ForEach(expContext => result += ", " + expContext.GetText());
            if (result.StartsWith(", "))
                result = result.Substring(", ".Length);
            return result;
        }

        public static string ParseStringExp(StringExpContext stringExp)
        {
            string result = stringExp.children[0].GetText();
            if (result.StartsWith("\""))
                result = result[1..];
            if (result.EndsWith("\""))
                result = result[..^1];
            return result;
        }

        public static float ParseNumberExp(NumberExpContext numberExp)
        {
            string numberStr = numberExp.children[0].GetText();
            bool valid = Double.TryParse(numberStr, out double result);
            if (!valid)
                throw new InvalidScriptException("Cannot parse number in: " + numberExp.Parent.GetText());
            return (float)result;
        }

        public static List<ExpressionContext> ParseArray(ArrayExpContext arrayExpContext)
        {
            if (arrayExpContext.exception != null)
                throw new InvalidScriptException("Cannot parse array: " + arrayExpContext.GetText() +
                    ". Exception: " + arrayExpContext.exception.Message);

            // if array is empty
            // arrayExpContext.children[0] is `[`
            if (arrayExpContext.children[1] is TerminalNodeImpl)
                return new List<ExpressionContext>();

            IParseTree argsNode = arrayExpContext.children[1];
            return ParseFuncArgs((ArgumentListContext)argsNode);
        }

        public static List<ExpressionContext> ParseFuncArgs(ArgumentListContext argsContext)
        {
            if (argsContext.exception != null)
                throw new InvalidScriptException("Cannot parse function arguments: " + argsContext.GetText() +
                    ". Exception: " + argsContext.exception.Message);

            List<ExpressionContext> result = new List<ExpressionContext>();
            if (argsContext.children == null)
                return result;
            if (argsContext.children[0] is ExpressionContext)
                result.Add((ExpressionContext)argsContext.children[0]);
            else
                throw new InvalidScriptException("Cannot parse argument: " + argsContext.children[0].GetText());

            for (int i = 1; i < argsContext.ChildCount; i += 2)
            {
                ParserUtils.ValidateAColon(argsContext.children[i]);
                if (argsContext.children[i + 1] is ExpressionContext)
                    result.Add((ExpressionContext)argsContext.children[i + 1]);
                else
                    throw new InvalidScriptException("Cannot parse: " + argsContext.children[i + 1].GetText() + " as a parameter of function");
            }
            return result;
        }
    }
}

