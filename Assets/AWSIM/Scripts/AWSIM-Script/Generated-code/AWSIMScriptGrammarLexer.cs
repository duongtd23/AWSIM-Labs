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

using System;
using System.IO;
using System.Text;
using Antlr4.Runtime;
using Antlr4.Runtime.Atn;
using Antlr4.Runtime.Misc;
using DFA = Antlr4.Runtime.Dfa.DFA;

[System.CodeDom.Compiler.GeneratedCode("ANTLR", "4.13.1")]
[System.CLSCompliant(false)]
public partial class AWSIMScriptGrammarLexer : Lexer {
	protected static DFA[] decisionToDFA;
	protected static PredictionContextCache sharedContextCache = new PredictionContextCache();
	public const int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, T__13=14, T__14=15, T__15=16, T__16=17, 
		T__17=18, T__18=19, T__19=20, T__20=21, T__21=22, T__22=23, T__23=24, 
		T__24=25, T__25=26, T__26=27, T__27=28, T__28=29, STRING=30, SIGN=31, 
		NUMBER=32, ID=33, WS=34, LINE_COMMENT=35;
	public static string[] channelNames = {
		"DEFAULT_TOKEN_CHANNEL", "HIDDEN"
	};

	public static string[] modeNames = {
		"DEFAULT_MODE"
	};

	public static readonly string[] ruleNames = {
		"T__0", "T__1", "T__2", "T__3", "T__4", "T__5", "T__6", "T__7", "T__8", 
		"T__9", "T__10", "T__11", "T__12", "T__13", "T__14", "T__15", "T__16", 
		"T__17", "T__18", "T__19", "T__20", "T__21", "T__22", "T__23", "T__24", 
		"T__25", "T__26", "T__27", "T__28", "STRING", "SIGN", "NUMBER", "ID", 
		"WS", "LINE_COMMENT"
	};


	public AWSIMScriptGrammarLexer(ICharStream input)
	: this(input, Console.Out, Console.Error) { }

	public AWSIMScriptGrammarLexer(ICharStream input, TextWriter output, TextWriter errorOutput)
	: base(input, output, errorOutput)
	{
		Interpreter = new LexerATNSimulator(this, _ATN, decisionToDFA, sharedContextCache);
	}

	private static readonly string[] _LiteralNames = {
		null, "'at'", "'back'", "'forward'", "'left'", "'right'", "'max-velocity'", 
		"'('", "')'", "'change-lane'", "'longitudinal-velocity'", "'lateral-velocity'", 
		"'velocity'", "','", "'with'", "'dx'", "'aggressive-driving'", "'acceleration'", 
		"'deceleration'", "'delay-spawn'", "'delay-move'", "'delay-spawn-until-ego-move'", 
		"'delay-move-until-ego-move'", "'delay-spawn-until-ego-engaged'", "'delay-move-until-ego-engaged'", 
		"'saving-timeout'", "'['", "']'", "'='", "';'"
	};
	private static readonly string[] _SymbolicNames = {
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, null, null, null, null, "STRING", "SIGN", "NUMBER", "ID", 
		"WS", "LINE_COMMENT"
	};
	public static readonly IVocabulary DefaultVocabulary = new Vocabulary(_LiteralNames, _SymbolicNames);

	[NotNull]
	public override IVocabulary Vocabulary
	{
		get
		{
			return DefaultVocabulary;
		}
	}

	public override string GrammarFileName { get { return "AWSIMScriptGrammar.g4"; } }

	public override string[] RuleNames { get { return ruleNames; } }

	public override string[] ChannelNames { get { return channelNames; } }

	public override string[] ModeNames { get { return modeNames; } }

	public override int[] SerializedAtn { get { return _serializedATN; } }

	static AWSIMScriptGrammarLexer() {
		decisionToDFA = new DFA[_ATN.NumberOfDecisions];
		for (int i = 0; i < _ATN.NumberOfDecisions; i++) {
			decisionToDFA[i] = new DFA(_ATN.GetDecisionState(i), i);
		}
	}
	private static int[] _serializedATN = {
		4,0,35,441,6,-1,2,0,7,0,2,1,7,1,2,2,7,2,2,3,7,3,2,4,7,4,2,5,7,5,2,6,7,
		6,2,7,7,7,2,8,7,8,2,9,7,9,2,10,7,10,2,11,7,11,2,12,7,12,2,13,7,13,2,14,
		7,14,2,15,7,15,2,16,7,16,2,17,7,17,2,18,7,18,2,19,7,19,2,20,7,20,2,21,
		7,21,2,22,7,22,2,23,7,23,2,24,7,24,2,25,7,25,2,26,7,26,2,27,7,27,2,28,
		7,28,2,29,7,29,2,30,7,30,2,31,7,31,2,32,7,32,2,33,7,33,2,34,7,34,1,0,1,
		0,1,0,1,1,1,1,1,1,1,1,1,1,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,3,1,3,1,3,
		1,3,1,3,1,4,1,4,1,4,1,4,1,4,1,4,1,5,1,5,1,5,1,5,1,5,1,5,1,5,1,5,1,5,1,
		5,1,5,1,5,1,5,1,6,1,6,1,7,1,7,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,
		1,8,1,8,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,9,1,
		9,1,9,1,9,1,9,1,9,1,9,1,9,1,10,1,10,1,10,1,10,1,10,1,10,1,10,1,10,1,10,
		1,10,1,10,1,10,1,10,1,10,1,10,1,10,1,10,1,11,1,11,1,11,1,11,1,11,1,11,
		1,11,1,11,1,11,1,12,1,12,1,13,1,13,1,13,1,13,1,13,1,14,1,14,1,14,1,15,
		1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,1,15,
		1,15,1,15,1,15,1,15,1,16,1,16,1,16,1,16,1,16,1,16,1,16,1,16,1,16,1,16,
		1,16,1,16,1,16,1,17,1,17,1,17,1,17,1,17,1,17,1,17,1,17,1,17,1,17,1,17,
		1,17,1,17,1,18,1,18,1,18,1,18,1,18,1,18,1,18,1,18,1,18,1,18,1,18,1,18,
		1,19,1,19,1,19,1,19,1,19,1,19,1,19,1,19,1,19,1,19,1,19,1,20,1,20,1,20,
		1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,
		1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,20,1,21,1,21,1,21,1,21,
		1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,
		1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,22,1,22,1,22,1,22,1,22,1,22,
		1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,
		1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,22,1,23,1,23,1,23,1,23,
		1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,
		1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,23,1,24,1,24,1,24,
		1,24,1,24,1,24,1,24,1,24,1,24,1,24,1,24,1,24,1,24,1,24,1,24,1,25,1,25,
		1,26,1,26,1,27,1,27,1,28,1,28,1,29,1,29,5,29,391,8,29,10,29,12,29,394,
		9,29,1,29,1,29,1,30,1,30,1,31,3,31,401,8,31,1,31,5,31,404,8,31,10,31,12,
		31,407,9,31,1,31,3,31,410,8,31,1,31,4,31,413,8,31,11,31,12,31,414,1,32,
		1,32,5,32,419,8,32,10,32,12,32,422,9,32,1,33,4,33,425,8,33,11,33,12,33,
		426,1,33,1,33,1,34,1,34,1,34,1,34,5,34,435,8,34,10,34,12,34,438,9,34,1,
		34,1,34,1,392,0,35,1,1,3,2,5,3,7,4,9,5,11,6,13,7,15,8,17,9,19,10,21,11,
		23,12,25,13,27,14,29,15,31,16,33,17,35,18,37,19,39,20,41,21,43,22,45,23,
		47,24,49,25,51,26,53,27,55,28,57,29,59,30,61,31,63,32,65,33,67,34,69,35,
		1,0,6,2,0,43,43,45,45,1,0,48,57,3,0,65,90,95,95,97,122,4,0,48,57,65,90,
		95,95,97,122,3,0,9,10,13,13,32,32,2,0,10,10,13,13,448,0,1,1,0,0,0,0,3,
		1,0,0,0,0,5,1,0,0,0,0,7,1,0,0,0,0,9,1,0,0,0,0,11,1,0,0,0,0,13,1,0,0,0,
		0,15,1,0,0,0,0,17,1,0,0,0,0,19,1,0,0,0,0,21,1,0,0,0,0,23,1,0,0,0,0,25,
		1,0,0,0,0,27,1,0,0,0,0,29,1,0,0,0,0,31,1,0,0,0,0,33,1,0,0,0,0,35,1,0,0,
		0,0,37,1,0,0,0,0,39,1,0,0,0,0,41,1,0,0,0,0,43,1,0,0,0,0,45,1,0,0,0,0,47,
		1,0,0,0,0,49,1,0,0,0,0,51,1,0,0,0,0,53,1,0,0,0,0,55,1,0,0,0,0,57,1,0,0,
		0,0,59,1,0,0,0,0,61,1,0,0,0,0,63,1,0,0,0,0,65,1,0,0,0,0,67,1,0,0,0,0,69,
		1,0,0,0,1,71,1,0,0,0,3,74,1,0,0,0,5,79,1,0,0,0,7,87,1,0,0,0,9,92,1,0,0,
		0,11,98,1,0,0,0,13,111,1,0,0,0,15,113,1,0,0,0,17,115,1,0,0,0,19,127,1,
		0,0,0,21,149,1,0,0,0,23,166,1,0,0,0,25,175,1,0,0,0,27,177,1,0,0,0,29,182,
		1,0,0,0,31,185,1,0,0,0,33,204,1,0,0,0,35,217,1,0,0,0,37,230,1,0,0,0,39,
		242,1,0,0,0,41,253,1,0,0,0,43,280,1,0,0,0,45,306,1,0,0,0,47,336,1,0,0,
		0,49,365,1,0,0,0,51,380,1,0,0,0,53,382,1,0,0,0,55,384,1,0,0,0,57,386,1,
		0,0,0,59,388,1,0,0,0,61,397,1,0,0,0,63,400,1,0,0,0,65,416,1,0,0,0,67,424,
		1,0,0,0,69,430,1,0,0,0,71,72,5,97,0,0,72,73,5,116,0,0,73,2,1,0,0,0,74,
		75,5,98,0,0,75,76,5,97,0,0,76,77,5,99,0,0,77,78,5,107,0,0,78,4,1,0,0,0,
		79,80,5,102,0,0,80,81,5,111,0,0,81,82,5,114,0,0,82,83,5,119,0,0,83,84,
		5,97,0,0,84,85,5,114,0,0,85,86,5,100,0,0,86,6,1,0,0,0,87,88,5,108,0,0,
		88,89,5,101,0,0,89,90,5,102,0,0,90,91,5,116,0,0,91,8,1,0,0,0,92,93,5,114,
		0,0,93,94,5,105,0,0,94,95,5,103,0,0,95,96,5,104,0,0,96,97,5,116,0,0,97,
		10,1,0,0,0,98,99,5,109,0,0,99,100,5,97,0,0,100,101,5,120,0,0,101,102,5,
		45,0,0,102,103,5,118,0,0,103,104,5,101,0,0,104,105,5,108,0,0,105,106,5,
		111,0,0,106,107,5,99,0,0,107,108,5,105,0,0,108,109,5,116,0,0,109,110,5,
		121,0,0,110,12,1,0,0,0,111,112,5,40,0,0,112,14,1,0,0,0,113,114,5,41,0,
		0,114,16,1,0,0,0,115,116,5,99,0,0,116,117,5,104,0,0,117,118,5,97,0,0,118,
		119,5,110,0,0,119,120,5,103,0,0,120,121,5,101,0,0,121,122,5,45,0,0,122,
		123,5,108,0,0,123,124,5,97,0,0,124,125,5,110,0,0,125,126,5,101,0,0,126,
		18,1,0,0,0,127,128,5,108,0,0,128,129,5,111,0,0,129,130,5,110,0,0,130,131,
		5,103,0,0,131,132,5,105,0,0,132,133,5,116,0,0,133,134,5,117,0,0,134,135,
		5,100,0,0,135,136,5,105,0,0,136,137,5,110,0,0,137,138,5,97,0,0,138,139,
		5,108,0,0,139,140,5,45,0,0,140,141,5,118,0,0,141,142,5,101,0,0,142,143,
		5,108,0,0,143,144,5,111,0,0,144,145,5,99,0,0,145,146,5,105,0,0,146,147,
		5,116,0,0,147,148,5,121,0,0,148,20,1,0,0,0,149,150,5,108,0,0,150,151,5,
		97,0,0,151,152,5,116,0,0,152,153,5,101,0,0,153,154,5,114,0,0,154,155,5,
		97,0,0,155,156,5,108,0,0,156,157,5,45,0,0,157,158,5,118,0,0,158,159,5,
		101,0,0,159,160,5,108,0,0,160,161,5,111,0,0,161,162,5,99,0,0,162,163,5,
		105,0,0,163,164,5,116,0,0,164,165,5,121,0,0,165,22,1,0,0,0,166,167,5,118,
		0,0,167,168,5,101,0,0,168,169,5,108,0,0,169,170,5,111,0,0,170,171,5,99,
		0,0,171,172,5,105,0,0,172,173,5,116,0,0,173,174,5,121,0,0,174,24,1,0,0,
		0,175,176,5,44,0,0,176,26,1,0,0,0,177,178,5,119,0,0,178,179,5,105,0,0,
		179,180,5,116,0,0,180,181,5,104,0,0,181,28,1,0,0,0,182,183,5,100,0,0,183,
		184,5,120,0,0,184,30,1,0,0,0,185,186,5,97,0,0,186,187,5,103,0,0,187,188,
		5,103,0,0,188,189,5,114,0,0,189,190,5,101,0,0,190,191,5,115,0,0,191,192,
		5,115,0,0,192,193,5,105,0,0,193,194,5,118,0,0,194,195,5,101,0,0,195,196,
		5,45,0,0,196,197,5,100,0,0,197,198,5,114,0,0,198,199,5,105,0,0,199,200,
		5,118,0,0,200,201,5,105,0,0,201,202,5,110,0,0,202,203,5,103,0,0,203,32,
		1,0,0,0,204,205,5,97,0,0,205,206,5,99,0,0,206,207,5,99,0,0,207,208,5,101,
		0,0,208,209,5,108,0,0,209,210,5,101,0,0,210,211,5,114,0,0,211,212,5,97,
		0,0,212,213,5,116,0,0,213,214,5,105,0,0,214,215,5,111,0,0,215,216,5,110,
		0,0,216,34,1,0,0,0,217,218,5,100,0,0,218,219,5,101,0,0,219,220,5,99,0,
		0,220,221,5,101,0,0,221,222,5,108,0,0,222,223,5,101,0,0,223,224,5,114,
		0,0,224,225,5,97,0,0,225,226,5,116,0,0,226,227,5,105,0,0,227,228,5,111,
		0,0,228,229,5,110,0,0,229,36,1,0,0,0,230,231,5,100,0,0,231,232,5,101,0,
		0,232,233,5,108,0,0,233,234,5,97,0,0,234,235,5,121,0,0,235,236,5,45,0,
		0,236,237,5,115,0,0,237,238,5,112,0,0,238,239,5,97,0,0,239,240,5,119,0,
		0,240,241,5,110,0,0,241,38,1,0,0,0,242,243,5,100,0,0,243,244,5,101,0,0,
		244,245,5,108,0,0,245,246,5,97,0,0,246,247,5,121,0,0,247,248,5,45,0,0,
		248,249,5,109,0,0,249,250,5,111,0,0,250,251,5,118,0,0,251,252,5,101,0,
		0,252,40,1,0,0,0,253,254,5,100,0,0,254,255,5,101,0,0,255,256,5,108,0,0,
		256,257,5,97,0,0,257,258,5,121,0,0,258,259,5,45,0,0,259,260,5,115,0,0,
		260,261,5,112,0,0,261,262,5,97,0,0,262,263,5,119,0,0,263,264,5,110,0,0,
		264,265,5,45,0,0,265,266,5,117,0,0,266,267,5,110,0,0,267,268,5,116,0,0,
		268,269,5,105,0,0,269,270,5,108,0,0,270,271,5,45,0,0,271,272,5,101,0,0,
		272,273,5,103,0,0,273,274,5,111,0,0,274,275,5,45,0,0,275,276,5,109,0,0,
		276,277,5,111,0,0,277,278,5,118,0,0,278,279,5,101,0,0,279,42,1,0,0,0,280,
		281,5,100,0,0,281,282,5,101,0,0,282,283,5,108,0,0,283,284,5,97,0,0,284,
		285,5,121,0,0,285,286,5,45,0,0,286,287,5,109,0,0,287,288,5,111,0,0,288,
		289,5,118,0,0,289,290,5,101,0,0,290,291,5,45,0,0,291,292,5,117,0,0,292,
		293,5,110,0,0,293,294,5,116,0,0,294,295,5,105,0,0,295,296,5,108,0,0,296,
		297,5,45,0,0,297,298,5,101,0,0,298,299,5,103,0,0,299,300,5,111,0,0,300,
		301,5,45,0,0,301,302,5,109,0,0,302,303,5,111,0,0,303,304,5,118,0,0,304,
		305,5,101,0,0,305,44,1,0,0,0,306,307,5,100,0,0,307,308,5,101,0,0,308,309,
		5,108,0,0,309,310,5,97,0,0,310,311,5,121,0,0,311,312,5,45,0,0,312,313,
		5,115,0,0,313,314,5,112,0,0,314,315,5,97,0,0,315,316,5,119,0,0,316,317,
		5,110,0,0,317,318,5,45,0,0,318,319,5,117,0,0,319,320,5,110,0,0,320,321,
		5,116,0,0,321,322,5,105,0,0,322,323,5,108,0,0,323,324,5,45,0,0,324,325,
		5,101,0,0,325,326,5,103,0,0,326,327,5,111,0,0,327,328,5,45,0,0,328,329,
		5,101,0,0,329,330,5,110,0,0,330,331,5,103,0,0,331,332,5,97,0,0,332,333,
		5,103,0,0,333,334,5,101,0,0,334,335,5,100,0,0,335,46,1,0,0,0,336,337,5,
		100,0,0,337,338,5,101,0,0,338,339,5,108,0,0,339,340,5,97,0,0,340,341,5,
		121,0,0,341,342,5,45,0,0,342,343,5,109,0,0,343,344,5,111,0,0,344,345,5,
		118,0,0,345,346,5,101,0,0,346,347,5,45,0,0,347,348,5,117,0,0,348,349,5,
		110,0,0,349,350,5,116,0,0,350,351,5,105,0,0,351,352,5,108,0,0,352,353,
		5,45,0,0,353,354,5,101,0,0,354,355,5,103,0,0,355,356,5,111,0,0,356,357,
		5,45,0,0,357,358,5,101,0,0,358,359,5,110,0,0,359,360,5,103,0,0,360,361,
		5,97,0,0,361,362,5,103,0,0,362,363,5,101,0,0,363,364,5,100,0,0,364,48,
		1,0,0,0,365,366,5,115,0,0,366,367,5,97,0,0,367,368,5,118,0,0,368,369,5,
		105,0,0,369,370,5,110,0,0,370,371,5,103,0,0,371,372,5,45,0,0,372,373,5,
		116,0,0,373,374,5,105,0,0,374,375,5,109,0,0,375,376,5,101,0,0,376,377,
		5,111,0,0,377,378,5,117,0,0,378,379,5,116,0,0,379,50,1,0,0,0,380,381,5,
		91,0,0,381,52,1,0,0,0,382,383,5,93,0,0,383,54,1,0,0,0,384,385,5,61,0,0,
		385,56,1,0,0,0,386,387,5,59,0,0,387,58,1,0,0,0,388,392,5,34,0,0,389,391,
		9,0,0,0,390,389,1,0,0,0,391,394,1,0,0,0,392,393,1,0,0,0,392,390,1,0,0,
		0,393,395,1,0,0,0,394,392,1,0,0,0,395,396,5,34,0,0,396,60,1,0,0,0,397,
		398,7,0,0,0,398,62,1,0,0,0,399,401,3,61,30,0,400,399,1,0,0,0,400,401,1,
		0,0,0,401,409,1,0,0,0,402,404,7,1,0,0,403,402,1,0,0,0,404,407,1,0,0,0,
		405,403,1,0,0,0,405,406,1,0,0,0,406,408,1,0,0,0,407,405,1,0,0,0,408,410,
		5,46,0,0,409,405,1,0,0,0,409,410,1,0,0,0,410,412,1,0,0,0,411,413,7,1,0,
		0,412,411,1,0,0,0,413,414,1,0,0,0,414,412,1,0,0,0,414,415,1,0,0,0,415,
		64,1,0,0,0,416,420,7,2,0,0,417,419,7,3,0,0,418,417,1,0,0,0,419,422,1,0,
		0,0,420,418,1,0,0,0,420,421,1,0,0,0,421,66,1,0,0,0,422,420,1,0,0,0,423,
		425,7,4,0,0,424,423,1,0,0,0,425,426,1,0,0,0,426,424,1,0,0,0,426,427,1,
		0,0,0,427,428,1,0,0,0,428,429,6,33,0,0,429,68,1,0,0,0,430,431,5,47,0,0,
		431,432,5,47,0,0,432,436,1,0,0,0,433,435,8,5,0,0,434,433,1,0,0,0,435,438,
		1,0,0,0,436,434,1,0,0,0,436,437,1,0,0,0,437,439,1,0,0,0,438,436,1,0,0,
		0,439,440,6,34,0,0,440,70,1,0,0,0,9,0,392,400,405,409,414,420,426,436,
		1,6,0,0
	};

	public static readonly ATN _ATN =
		new ATNDeserializer().Deserialize(_serializedATN);


}
