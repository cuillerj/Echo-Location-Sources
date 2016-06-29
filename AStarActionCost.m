function [cost] = AStarActionCost(currentAction,nextAction,stepSize)
	moveX=nextAction(1);
	moveY=nextAction(2);
	cost=sqrt(moveX^2+moveY^2);  
	if (currentAction==[0,0])
		return
	endif
	if (currentAction==nextAction)
		cost=cost;
		return
	else
	%	cost=cost;
		cost=cost+stepSize*0.01;   % 0.01 tunned to add some cost to reduce the number of move kind changes - to smooth the path
		return

	endif
	
endfunction