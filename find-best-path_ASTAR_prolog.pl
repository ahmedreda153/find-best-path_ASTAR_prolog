% To start the program, call predicate 1 or 2:
% 1. start_search([1,1,red], [2,3,red], 3, 3, [[1,1,red], [1,2,red], [1,3,yellow], [2,1,red], [2,2,blue], [2,3,red], [3,1,red], [3,2,red], [3,3,red]]).
% 2. start_search([1,1,red], [2,4,red], 4, 4, [[1,1,red], [1,2,red], [1,3,yellow], [1,4,yellow], [2,1,red], [2,2,blue], [2,3,red], [2,4,red], [3,1,red], [3,2,red], [3,3,red], [3,4,yellow], [4,1,blue], [4,2,red], [4,3,blue], [4,4,yellow]]).

% This program implements a pathfinding algorithm using Greedy Best-First Search for a grid with colored nodes.
% The algorithm operates as follows:
% 1. Initialize with the start node, exploring adjacent nodes with the least heuristic cost.
% 2. Track all visited nodes to prevent revisits and ensure pathfinding.
% 3. Upon reaching the goal node, report the path from start to goal.
% 4. Report the presence of a path along with its path.
% 5. If no path is found, report that no paths exist.

% Entry point to start pathfinding in a grid.
start_search(Start, Goal, N, M, Grid):-
    search([[Start, null, 0, 0, 0]], [], Goal,N, M, Grid).
start_search(_, _, _, _, _):-
    write("No Path Found!"), nl.

% Checks if the path is found by comparing the current node with the goal.
search(Queue, Visited, Goal,_,_,_):-
    getBestState(Queue, [CurrentState,Parent,G,H,F], _),
    CurrentState = Goal,
    write("Path is Found!"), nl,
    printSolution([CurrentState,Parent,G,H,F], Visited), !.

% Performs Greedy Best-First Search to find the path.
search(Queue, Visited, Goal,N,M,Grid):-
    getBestState(Queue, CurrentNode, TmpQueue),
    getAllValidChildren(CurrentNode,TmpQueue,Visited,Goal,Children,N,M,Grid),
    addChildren(Children, TmpQueue, NewQueue),
    append(Visited, [CurrentNode], NewVisited),
    search(NewQueue, NewVisited, Goal,N,M,Grid).

% Gets the best state from the queue.
getBestState(Queue, BestChild, Rest):-
    findMin(Queue, BestChild),
    delete(Queue, BestChild, Rest).

% Finds the minimum F value in the queue.
findMin([X], X):- !.
findMin([Head|T], Min):-
    findMin(T, TmpMin),
    Head = [_,_,_,_,HeadF],
    TmpMin = [_,_,_,_,TmpF],
    (TmpF < HeadF -> Min = TmpMin ; Min = Head).

% Gets all valid children from the current state.
getAllValidChildren(Node, Queue, Visited, Goal, Children,N,M,Grid):-
    findall(Next, getNextState(Node,Queue,Visited,Goal,Next,N,M,Grid), Children).

% Adds children to the queue.
addChildren(Children, Queue, NewQueue):-
    append(Queue, Children, NewQueue).

% Gets all valid next states from the current state.
getNextState([State,_,G,_,_],Queue,Visited,Goal,[NextNode,State,NewG,NewH,NewF],N,M,Grid):-
    move(State, Next, MoveCost,N,M),
    calculateH(Next, Goal, NewH),
    NewG is G + MoveCost,
    NewF is NewG + NewH,
    search_getColor(Next, Grid, NextNode),
    checkColor(State, NextNode),
    ( not(member([NextNode,_,_,_,_], Queue)); 
    memberButBetter(NextNode,Queue,NewF) ),
    ( not(member([NextNode,_,_,_,_],Visited)); 
    memberButBetter(NextNode,Visited,NewF)).

% Movement rules within the grid.
% All moves in this problem have equal cost (0)
move(State, Next, 0, N, M):- 
    left(State, Next,N); 
    right(State, Next,N);
    up(State, Next,M);
    down(State, Next,M).

left([X,Y,_], [X1,Y1,_], M):-
    Y1 is Y-1, X1 is X, Y1 > 0, Y1 =< M.

right([X,Y,_], [X1,Y1,_], M):-
    Y1 is Y+1, X1 is X, Y1 > 0, Y1 =< M.

up([X,Y,_], [X1,Y1,_], N):-
    X1 is X-1, Y1 is Y, X1 > 0, X1 =< N.

down([X,Y,_], [X1,Y1,_], N):-
    X1 is X+1, Y1 is Y, X1 > 0, X1 =< N.

% Heuristic function to calculate the Manhattan distance between two nodes.
calculateH([], [], 0):- !.
calculateH([X1,Y1,_], [X2,Y2,_], H):-
    H is abs(X1-X2) + abs(Y1-Y2).

% Finds the color of a node given its coordinates.
search_getColor(_, [], _):- fail.
search_getColor([X, Y, _], [[X1, Y1, Color]|_], Result) :-
    X is X1, Y is Y1, Result = [X1, Y1, Color],
    !.
search_getColor([X, Y, _], [[_, _, _]|T], Result) :-
    search_getColor([X, Y, _], T, Result).

% Helper to check if two nodes have the same color.
checkColor([_, _, Color], [_, _, Color1]):- 
    Color == Color1.

memberButBetter(Next, List, NewF):-
    findall(F, member([Next,_,_,_,F], List), Numbers),
    min_list(Numbers, MinOldF),
    MinOldF > NewF.

% Prints the solution path recursively.
printSolution([State, null, _, _, _],_):-
    write(State), nl.
printSolution([State, Parent, _, _, _], Visited):-
    member([Parent, GrandParent, PrevG, Ph, Pf], Visited),
    printSolution([Parent, GrandParent, PrevG, Ph, Pf], Visited),
    write(State), nl.