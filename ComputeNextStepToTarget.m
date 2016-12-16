function [nextX,nextY,rotation,distance,direct,forward] = ComputeNextStepToTarget(carto,currentX,currentY,currentHeading,targetX,targetY,newTarget,plotOn,robot,parametersNameList)
% first try direct acces
nextX=targetX;
nextY=targetY;
rotation=0;
currentHeading=mod(currentHeading+360,360);;
forward=0;
[rotation,distance,possible] = CheckStraightMovePossibility(carto,currentX,currentY,currentHeading,targetX,targetY);
if (possible==true)
	direct=true;
	printf(mfilename);
	printf(" *** go straight possible . *** ");
	printf(ctime(time()))
	direction=[targetX-currentX,targetY-currentY];
	currentHeadingGrad=currentHeading*pi()/180;
	vectHeading=[cos(currentHeadingGrad),sin(currentHeadingGrad)];
	projectionHeading=direction*vectHeading';
	if (projectionHeading >=0)
		forward=1;
	else
		forward=-1;
	endif
	return
else
	direct=false;
	printf(mfilename);
	printf(" *** need to compute path . *** ");
	printf(ctime(time()))
endif
% in case direct acces not possible determine path
if (newTarget==true)
	[AStarPath,AStarStep,cost,forward] = AStarSearch(carto,RoundTo(currentX,5),RoundTo(currentY,5),currentHeading,RoundTo(targetX,5),RoundTo(targetY,5),0,plotOn,robot,parametersNameList);
	if (cost<0)   % no path found
		forward=0;
		return
	endif
	[AStarStep,forward] = SmoothPath(carto,AStarPath,AStarStep,forward,plotOn);
	AStarStep
	save ("-mat4-binary","AStarStep.mat","AStarStep");
	nextX=AStarStep(2,1);
	nextY=AStarStep(2,2);
else
	load("AStarStep.mat");
	prevDist=inf;
	closest=0;
	for i=1:size(AStarStep,1)
		dist=sqrt((AStarStep(i,1)-currentX)^2+(AStarStep(i,2)-currentY)^2);
		if (dist<=prevDist)
			[rotation,distance,possible] = CheckStraightMovePossibility(carto,currentX,currentY,currentHeading,AStarStep(i,1),AStarStep(i,2));
			if (possible==1)
				closest=i;
				prevDist=dist;
			endif
		endif
	endfor
	printf(mfilename);
	printf(" *** closest step is:%d . *** ",closest);
	printf(ctime(time()));
	if (closest!=0)
		nextX=AStarStep(min(size(AStarStep,1),closest+1),1);
		nextY=AStarStep(min(size(AStarStep,1),closest+1),2);
	else
	[AStarPath,AStarStep,cost,forward] = AStarSearch(carto,RoundTo(currentX,5),RoundTo(currentY,5),currentHeading,RoundTo(targetX,5),RoundTo(targetY,5),0,plotOn,robot,parametersNameList);
	if (cost<0)   % no path found
		forward=0;
		return
	endif
	[AStarStep,forward] = SmoothPath(carto,AStarPath,AStarStep,forward,plotOn);
	save ("-mat4-binary","AStarStep.mat","AStarStep");
	nextX=AStarStep(2,1);
	nextY=AStarStep(2,2);
	endif
endif
direction=[nextX-currentX,nextY-currentY];
currentHeadingGrad=currentHeading*pi()/180;
vectHeading=[cos(currentHeadingGrad),sin(currentHeadingGrad)];
projectionHeading=direction*vectHeading';
if (projectionHeading >=0)
	forward=1;
else
	forward=-1;
endif
endfunction