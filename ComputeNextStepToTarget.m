function [nextX,nextY,rotation,distance,direct,startHeading,forward] = ComputeNextStepToTarget(carto,currentX,currentY,currentHeading,targetX,targetY,plotOn)
% first try direct acces
nextX=0;
nextY=0;
rotation=0;
startHeading=-1;
forward=1;
[rotation,distance,possible] = CheckStraightMovePossibility(carto,currentX,currentY,currentHeading,targetX,targetY);
if (possible==true)
	direct=true;
	printf("go straight possible .\n");
	return
else
	printf("need to compute path .\n");
endif
% in case direct acces not possible determine path
[AStarPath,AStarStep,cost,startHeading,forward] = AStarSearch(carto,RoundTo(currentX,5),RoundTo(currentY,5),currentHeading,RoundTo(targetX,5),RoundTo(targetY,5),0,plotOn);
if (cost<0)   % no path found
	forward=0;
	return
endif
[a,b]=size(AStarStep)
i=1;
prevPath=0;
nextPath=prevPath;
endPath=false;
direct=false;
while (prevPath==nextPath && endPath==false)
	prevPath=AStarPath(i);
	nextX=AStarStep(a-i,1);
	nextY=AStarStep(a-i,2);
	if (i>=a-1)
		endPath=true
	else
		nextPath=AStarPath(i+1);
		i=i+1;
	endif
end
endfunction