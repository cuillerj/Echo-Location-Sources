function [apRobot,robot,next,direct,forward] = ApComputeNextStepToTarget(apRobot,robot,plotOn)
%{ 
  compute next step to get the target (robot as a single point)
  first try to reach target in direct acces
  this is a new target use astar search to find a path and store this path
  look for the closest next stored step
  determine forward or backard
      forward -9 no step found
      forward 0 no move constraint
      forward 1 need to go forward
      forward -1 need to go backward
%}
currentL=apGet(apRobot,"location");
targetL=apGet(apRobot,"destination");
newTarget=apGet(apRobot,"newTarget");      % true first request for this new target
apRobot = setfield(apRobot,"newTarget",false);  %
pathMaxStraightLenght=apGet(apRobot,"pathMaxStraightLenght");
stepSize=apGet(apRobot,"stepSize");
carto=apGet(apRobot,"carto");
nextX=targetL(1);
nextY=targetL(2);
rotation=0;
currentX=RoundTo(currentL(1),5);
currentY=RoundTo(currentL(2),5);
currentHeading=mod(currentL(3)+360,360);
targetX=RoundTo(targetL(1),5);
targetY=RoundTo(targetL(2),5);
forward=0;
instruction=[];
direct=false;

if (newTarget==false && (sqrt((currentL(1)-targetL(1))^2+(currentL(2)-targetL(2))^2)<=pathMaxStraightLenght))
  [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,currentL,targetL);
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
    next=[nextX,nextY];
    return
  else
    printf(mfilename);
    printf(" *** need to compute path . *** ");
    printf(ctime(time()))
  endif
endif
% in case direct acces not possible determine path
if (newTarget==true)
 % [apRobot,robot,AStarStep,cost,forward] = ApComputeOptimalPath(apRobot,robot,targetL,plotOn)
 %{
	[apRobot,robot,AStarPath,AStarStep,cost,forward] = ApAStarSearch(apRobot,robot,currentL,targetL,plotOn);
	if (cost<0)   % no path found
		forward=0;
    next=[currentL(1),currentL(2)];
		return
	endif
	[apRobot,AStarStep,forward] = ApSmoothPath(apRobot,AStarPath,AStarStep,forward,plotOn);
	save ("-mat4-binary","AStarStep.mat","AStarStep");
	nextX=AStarStep(2,1);
	nextY=AStarStep(2,2);
  %}
else
%	load("AStarStep.mat");
  %load(AStarStepSmooth.mat);
  AStarStep=apGet(apRobot,"pathStep");
	prevDist=Inf;
	closest=1;
  if (size(AStarStep,1)<=1)
    next=target
    direect=true;
    printf(mfilename);
    printf(" *** next step is target *** ");
    printf(ctime(time()));
    return
  endif


    
	for i=1:size(AStarStep,1)
		dist=sqrt((AStarStep(i,1)-currentX)^2+(AStarStep(i,2)-currentY)^2);
		if (dist<=prevDist)
			[apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,currentL,[AStarStep(i,1),AStarStep(i,2),0]);
			if (possible==1)
					[apRobot,rotation2,distance2,possible2] = ApCheckStraightMovePossibility(apRobot,currentL,[AStarStep(i+1,1),AStarStep(i+1,2),0]);
					if (possible2==1)
						closest=i+1;
					else
						closest=i;
					endif
				prevDist=dist;
			endif
		endif
	endfor
	printf(mfilename);
	printf(" *** closest step is:%d . *** ",closest);
	printf(ctime(time()));
	if (closest!=0)
    if (sqrt((AStarStep(closest,1)-currentX)^2+(AStarStep(closest,2)-currentY)^2) < apGet(apRobot,"minDistToBeDone"))
     % AStarStep=AStarStep(2:size(AStarStep,1),:); % suppress first step too close
      closest++;
      printf(mfilename);
      printf(" *** first step too close *** ");
      printf(ctime(time()));
    endif
    if (newTarget==false)
		  nextX=AStarStep(min(size(AStarStep,1),closest),1);
		  nextY=AStarStep(min(size(AStarStep,1),closest),2);
      instruction=AStarStep(min(size(AStarStep,1),closest),4);
    else
    	nextX=AStarStep(1,1);
		  nextY=AStarStep(1,2);
      instruction=AStarStep(1,4);
    endif
	else
	 [apRobot,robot,AStarPath,AStarStep,cost] = ApAStarSearch(apRobot,robot,currentL,[RoundTo(targetX,5),RoundTo(targetY,5),0],plotOn);
  	if (cost<0)   % no path found
		  forward=0;
      next=[nextX,nextY];
		  return
	  endif
    if (newTarget==false) 
	    [apRobot,AStarStep,forward] = ApSmoothPath(apRobot,AStarPath,AStarStep,forward,plotOn);
    endif
	  save ("-mat4-binary","AStarStep.mat","AStarStep");
	  nextX=AStarStep(2,1);
	  nextY=AStarStep(2,2);
	endif
endif
direction=[nextX-currentX,nextY-currentY];
currentHeadingGrad=currentHeading*pi()/180;
vectHeading=[cos(currentHeadingGrad),sin(currentHeadingGrad)];
projectionHeading=direction*vectHeading';
%{
if (projectionHeading >=0)
	forward=1;
else
	forward=-1;
endif
%}
next=[nextX,nextY];
forward=instruction;
endfunction