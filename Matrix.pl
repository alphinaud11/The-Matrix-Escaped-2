:-include('KB.pl').

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Base case.


matrix_state([X,Y],HostagesState,s0):-
	neo_loc(X,Y),
	hostages_loc(HostagesList),
	length(HostagesList,N),
	length(HostagesState,N),
	maplist(=(n),HostagesState).
	
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% All possible conditions for when the action is drop.


matrix_state([X,Y],[d,d],result(drop,S)):-
	booth(X,Y), matrix_state([X,Y],[d,c],S).
	
matrix_state([X,Y],[d,d],result(drop,S)):-
	booth(X,Y), matrix_state([X,Y],[c,d],S).
	
matrix_state([X,Y],[d,n],result(drop,S)):-
	booth(X,Y), matrix_state([X,Y],[c,n],S).
	
matrix_state([X,Y],[n,d],result(drop,S)):-
	booth(X,Y), matrix_state([X,Y],[n,c],S).
	
matrix_state([X,Y],[d,d],result(drop,S)):-
	booth(X,Y), capacity(2), matrix_state([X,Y],[c,c],S).
	
matrix_state([X,Y],[d],result(drop,S)):-
	booth(X,Y), matrix_state([X,Y],[c],S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% All possible conditions for when the action is carry.

	
matrix_state([X,Y],[d,c],result(carry,S)):-
	hostages_loc([_,[X,Y]]), matrix_state([X,Y],[d,n],S).
	
matrix_state([X,Y],[c,d],result(carry,S)):-
	hostages_loc([[X,Y],_]), matrix_state([X,Y],[n,d],S).
	
matrix_state([X,Y],[c,n],result(carry,S)):-
	hostages_loc([[X,Y],_]), matrix_state([X,Y],[n,n],S).
	
matrix_state([X,Y],[n,c],result(carry,S)):-
	hostages_loc([_,[X,Y]]), matrix_state([X,Y],[n,n],S).
	
matrix_state([X,Y],[c,c],result(carry,S)):-
	hostages_loc([_,[X,Y]]), matrix_state([X,Y],[c,n],S).
	
matrix_state([X,Y],[c,c],result(carry,S)):-
	hostages_loc([[X,Y],_]), matrix_state([X,Y],[n,c],S).
	
matrix_state([X,Y],[c],result(carry,S)):-
	hostages_loc([[X,Y]]), matrix_state([X,Y],[n],S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% All possible conditions for when the action is up.

	
matrix_state([X,Y],HostagesState,result(up,S)):-
	grid(_,Height), N is Height - 1, X \= N, OldX is X + 1, matrix_state([OldX,Y],HostagesState,S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% All possible conditions for when the action is down.


matrix_state([X,Y],HostagesState,result(down,S)):-
	X \= 0, OldX is X - 1, matrix_state([OldX,Y],HostagesState,S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% All possible conditions for when the action is left.

	
matrix_state([X,Y],HostagesState,result(left,S)):-
	grid(Width,_), M is Width - 1, Y \= M, OldY is Y + 1, matrix_state([X,OldY],HostagesState,S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% All possible conditions for when the action is right.

		
matrix_state([X,Y],HostagesState,result(right,S)):-
	Y \= 0, OldY is Y - 1, matrix_state([X,OldY],HostagesState,S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% A predicate to help us do iterative deepening search inside goal(S).


goalHelper([X,Y],HostagesState,D,S):-
	(call_with_depth_limit(matrix_state([X,Y],HostagesState,S),D,R), R \= depth_limit_exceeded) ; (D1 is D + 1, goalHelper([X,Y],HostagesState,D1,S)).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% goal(S) for generating a plan.


goal(S):-
	var(S),
	booth(X,Y),
	hostages_loc(HostagesList),
	length(HostagesList,N),
	length(HostagesState,N),
	maplist(=(d),HostagesState),
	goalHelper([X,Y],HostagesState,0,S).

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% goal(S) for checking if a given plan is valid or not.


goal(S):-
	nonvar(S),
	booth(X,Y),
	hostages_loc(HostagesList),
	length(HostagesList,N),
	length(HostagesState,N),
	maplist(=(d),HostagesState),
	matrix_state([X,Y],HostagesState,S).