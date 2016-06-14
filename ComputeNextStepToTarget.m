function [nextX,nextY] = ComputeNextStepToTarget(currentX,currentY,targetX,targetY,plotOn)
% to be developp to find path
[AStarPath,AStarStep,cost] = AStarSearch(RoundTo(currentX,5),RoundTo(currentY,5),RoundTo(targetX,5),RoundTo(targetY,5),plotOn)
[a,b]=size(AStarStep)
i=1;
prevPath=0;
nextPath=prevPath;
endPath=false;
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