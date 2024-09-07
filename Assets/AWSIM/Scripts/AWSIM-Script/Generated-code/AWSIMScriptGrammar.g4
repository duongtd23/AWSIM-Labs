grammar AWSIMScriptGrammar;

// to specify a (2D) position
// e.g., `"TrafficLane.239" at 3.5` expresses the position on lane 239, 3.5m from the starting point of the lane.
// `positionA back 2` denotes the position behind positionA 2 meters
// `positionA left -2` denotes the position on the left lane of the lane where positionA is located and 2 meters backward shifted
positionExp
    : stringExp ('at' numberExp)? // pair of lane and offset
    | variableExp 'back' numberExp
    | variableExp 'forward' numberExp
    | variableExp 'left' numberExp
    | variableExp 'right' numberExp
    | positionExp 'back' numberExp
    | positionExp 'forward' numberExp
    | positionExp 'left' numberExp
    | positionExp 'right' numberExp;
roadExp
    : stringExp ('max-velocity' '(' numberExp ')')?
    | 'change-lane' ('at' numberExp)? ('longitudinal-velocity' '(' numberExp ')')? ('lateral-velocity' '(' numberExp ')')?
    | 'change-lane' ('at' numberExp)? 'velocity' '(' numberExp ',' numberExp ')'? ;
configExp
    : 'aggressive-driving'
    | 'acceleration' '(' numberExp ')'
    | 'deceleration' '(' numberExp ')'
    | 'delay-spawn' '(' numberExp ')'
    | 'delay-move' '(' numberExp ')'
    | 'delay-spawn-until-ego-move' '(' numberExp ')'
    | 'delay-move-until-ego-move' '(' numberExp ')'
    | 'delay-spawn-until-ego-engaged' '(' numberExp ')'
    | 'delay-move-until-ego-engaged' '(' numberExp ')';
egoSettingExp
    : 'max-velocity' '(' numberExp ')';
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