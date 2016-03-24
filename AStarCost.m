function [cost] = AStarCost(currentAction,nextAction)
xmov=[currentAction(1)-nextAction(1)];
ymov=[currentAction(2)-nextAction(2)];
cost=abs(xmov)+abs(ymov)+1;
endfunction