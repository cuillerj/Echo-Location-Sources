function [cost] = AStarActionCost(currentAction,nextAction,stepSize)
	maxN=abs(max(nextAction))*1.1; % * 1.1 tunned to add a little bit cost to the diagonal path
	minN=abs(min(nextAction))*1.1;   % * 1.1 tunned to add a little bit cost to the diagonal path
%	cost=max(maxN,minN);
	cost=sqrt(maxN^2+minN^2);
	if (currentAction==[0,0])
		return
	endif
	if (currentAction==nextAction)
		cost=cost;
		return
	else
		cost=cost+stepSize/2;   % tunned to add some cost to reduce the number of move kind changes
		return

	endif
	


endfunction