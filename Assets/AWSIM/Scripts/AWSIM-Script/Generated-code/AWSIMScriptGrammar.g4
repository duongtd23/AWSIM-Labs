grammar AWSIMScriptGrammar;

// to specify a (2D) position
// e.g., `"TrafficLane.239" at 3.5` expresses the position on lane 239, 3.5m from the starting point of the lane.
// `positionA back 2` denotes the position behind positionA 2 meters
// `positionA left -2` denotes the position on the left lane of the lane where positionA is located and 2 meters backward shifted
positionExp
    : stringExp ('at' (numberExp | variableExp))? // pair of lane and offset
    | variableExp 'back' (numberExp | variableExp)
    | variableExp 'forward' (numberExp | variableExp)
    | variableExp 'left' (numberExp | variableExp)
    | variableExp 'right' (numberExp | variableExp)
    | positionExp 'back' (numberExp | variableExp)
    | positionExp 'forward' (numberExp | variableExp)
    | positionExp 'left' (numberExp | variableExp)
    | positionExp 'right' (numberExp | variableExp);
roadExp
    : stringExp ('max-velocity' '(' (numberExp | variableExp) ')')?
    | 'change-lane' '(' argumentList? ')'
    | 'cut-in' '(' argumentList? ')' 
    | 'cut-out' '(' argumentList? ')';
configExp
    : 'aggressive-driving'
    | 'acceleration' '(' (numberExp | variableExp) ')'
    | 'deceleration' '(' (numberExp | variableExp) ')'
    | 'speed' '(' (numberExp | variableExp) ')'
    | 'delay-spawn' '(' (numberExp | variableExp) ')'
    | 'delay-move' '(' (numberExp | variableExp) ')'
    | 'delay-spawn-until-ego-move' '(' (numberExp | variableExp) ')'
    | 'delay-move-until-ego-move' '(' (numberExp | variableExp) ')'
    | 'delay-spawn-until-ego-engaged' '(' (numberExp | variableExp) ')'
    | 'delay-move-until-ego-engaged' '(' (numberExp | variableExp) ')';
vector2Exp
    : ((numberExp | variableExp) '#' (numberExp | variableExp));
egoSettingExp
    : 'max-velocity' '(' (numberExp | variableExp) ')';
simulationSettingExp
    : 'saving-timeout' '(' numberExp ')';
functionExp
    : idExp '(' argumentList? ')' ;
arrayExp
    : '[' argumentList? ']';
argumentList
    : expression ( ',' expression )* ;
assignmentStm
    : variableExp '=' expression;
// variable name, e.g., npc1
variableExp: idExp;
expression
    : stringExp
    | numberExp
    | vector2Exp
    | positionExp
    | roadExp
    | arrayExp
    | variableExp
    | configExp
    | egoSettingExp
    | simulationSettingExp
    | functionExp;
statement
    : (assignmentStm | functionExp) ';' ;
scenario
    : (statement)+ EOF;
stringExp
    : STRING;
numberExp
    : NUMBER;
idExp
    : ID;
// number and string data types
STRING : '"' .*? '"';
SIGN
    : ('+' | '-');
NUMBER
    : SIGN? ( [0-9]* '.' )? [0-9]+;

ID  : [a-zA-Z_] [a-zA-Z0-9_]*;

// ignore space(s)
WS  : (' '|'\t'|'\r'|'\n')+ -> skip;

LINE_COMMENT
    : '//' ~[\r\n]* -> skip;