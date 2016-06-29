function [cost] = AStarActionCost(currentAction,nextAction,stepSize)
	moveX=nextAction(1);
	moveY=nextAction(2);
	cost=sqrt(moveX^2+moveY^2+moveX*moveY);  % moveX*moveY to had some cost to diagonal move
	if (currentAction==[0,0])
		return
	endif
	if (currentAction==nextAction)
		cost=cost;
		return
	else
	%	cost=cost;
		cost=cost+stepSize*0.5;   % 0.1 tunned to add some cost to reduce the number of move kind changes
		return

	endif
	


endfunction