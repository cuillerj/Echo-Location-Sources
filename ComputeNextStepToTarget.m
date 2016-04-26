function [nextX,nextY] = ComputeNextStepToTarget(currentX,currentY,targetX,targetY,plotOn)
% to be developp to find path
[AStarPath,AStarStep,cost] = AStarSearch(currentX,currentY,targetX,targetY,plotOn);
[a,b]=size(AStarStep);
i=1;
prevPath=0;
nextPath=prevPath;
while (prevPath==nextPath)
	prevPath=AStarPath(i);
	nextPath=AStarPath(i+1);
	nextX=AStarStep(a-i,1);
	nextY=AStarStep(a-i,2);
	i=i+1;
end
endfunction