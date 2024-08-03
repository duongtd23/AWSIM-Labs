//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     ANTLR Version: 4.13.1
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

// Generated from ../grammar/AWSIMScriptGrammar.g4 by ANTLR 4.13.1

// Unreachable code detected
#pragma warning disable 0162
// The variable '...' is assigned but its value is never used
#pragma warning disable 0219
// Missing XML comment for publicly visible type or member '...'
#pragma warning disable 1591
// Ambiguous reference in cref attribute
#pragma warning disable 419

using Antlr4.Runtime.Misc;
using Antlr4.Runtime.Tree;
using IToken = Antlr4.Runtime.IToken;

/// <summary>
/// This interface defines a complete generic visitor for a parse tree produced
/// by <see cref="AWSIMScriptGrammarParser"/>.
/// </summary>
/// <typeparam name="Result">The return type of the visit operation.</typeparam>
[System.CodeDom.Compiler.GeneratedCode("ANTLR", "4.13.1")]
[System.CLSCompliant(false)]
public interface IAWSIMScriptGrammarVisitor<Result> : IParseTreeVisitor<Result> {
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.scenario"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitScenario([NotNull] AWSIMScriptGrammarParser.ScenarioContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.statement"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitStatement([NotNull] AWSIMScriptGrammarParser.StatementContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.assignmentStm"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitAssignmentStm([NotNull] AWSIMScriptGrammarParser.AssignmentStmContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.expression"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitExpression([NotNull] AWSIMScriptGrammarParser.ExpressionContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.functionExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitFunctionExp([NotNull] AWSIMScriptGrammarParser.FunctionExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.argumentList"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitArgumentList([NotNull] AWSIMScriptGrammarParser.ArgumentListContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.arrayExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitArrayExp([NotNull] AWSIMScriptGrammarParser.ArrayExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.positionExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitPositionExp([NotNull] AWSIMScriptGrammarParser.PositionExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.lanePositionExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitLanePositionExp([NotNull] AWSIMScriptGrammarParser.LanePositionExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.routeExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitRouteExp([NotNull] AWSIMScriptGrammarParser.RouteExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.variableExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitVariableExp([NotNull] AWSIMScriptGrammarParser.VariableExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.spawnDelayOptionExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitSpawnDelayOptionExp([NotNull] AWSIMScriptGrammarParser.SpawnDelayOptionExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.stringExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitStringExp([NotNull] AWSIMScriptGrammarParser.StringExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.numberExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitNumberExp([NotNull] AWSIMScriptGrammarParser.NumberExpContext context);
	/// <summary>
	/// Visit a parse tree produced by <see cref="AWSIMScriptGrammarParser.idExp"/>.
	/// </summary>
	/// <param name="context">The parse tree.</param>
	/// <return>The visitor result.</return>
	Result VisitIdExp([NotNull] AWSIMScriptGrammarParser.IdExpContext context);
}
