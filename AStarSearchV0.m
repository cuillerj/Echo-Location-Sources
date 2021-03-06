function [AStarPath,AStarStep,cost,startHeading,forward] = AStarSearchV0(currentX,currentY,currentHeading,targetX,targetY,targetHeading,plotOn)
%{ 
 AStarPath will return the ordered list of actions to go from current to target position
 AStarStep will return the list of positions touched to go from current to target position(ordered from target position to  current)
		step will be ploted if plotOn is true
 cost will return cost to go from current to target position
  %}
 % printf("astar search\n")
 currentHeading=mod(currentHeading+360,360);
 startHeading=-1;
 forward=0;
 currentHeadingGrad=currentHeading*pi()/180;
 targetHeadingGrad=targetHeading*pi()/180;
  AStarPath=[];
  AStarStep=[];
  stepSize=10;       % n lenght of the first kind of action must match with carto square size
  currentX=stepSize*(floor(currentX/stepSize))+floor(stepSize/2); % adjust position to center of square carto
  currentY=stepSize*(floor(currentY/stepSize))+floor(stepSize/2);
  targetX=stepSize*(floor(targetX/stepSize))+floor(stepSize/2);
  targetY=stepSize*(floor(targetY/stepSize))+floor(stepSize/2);
[angleOneHole,distanceOneHole] = StepEncoderByHole();
cartoId=1;        %  multiple cartography to be supported later
load carto1;       % cartographie matrix (1x1 cm)
carto=carto1;
[a,b]=size(carto); % a nb of X positions b nb of Y poistions
initPos=[currentX,currentY];
pos=initPos;
targetPos=[targetX,targetY];
%maxCartoWeight=20; % threshold over that means space (x,y) is not available
weightCarto=5;     % weight used to balance way depending on cartography weight and action cost

actions=[
		[stepSize,0];[-stepSize,0];
		[0,stepSize];[0,-stepSize];
		[stepSize,stepSize]; [stepSize,-stepSize];  % actions list (deltaX,deltaY)
		[-stepSize,stepSize];[-stepSize,-stepSize];
		];
actionsRotation=[0,-pi(),pi()/2,-pi()/2,pi()/4,-pi()/4,pi()*3/4,-pi()*3/4];
AStarHeading=currentHeadingGrad;
nbActions=size(actions,1);
currentAction=[0,0];
g=0;
currentCost=0;
%{ openList contains (X,Y) points that are to be developped
%}
openList=[g,pos(1),pos(2),currentAction,currentCost+AStarHeuristic(currentX,currentY,targetX,targetY),currentCost,AStarHeading];
AStarPath=[];         
AStarStep=[];
        
actionSteps=zeros(a,b);  % will keep for each cartography point the last action to go there
closed=zeros(a,b);       % will contains cartography points status
expandList=ones(a,b).*-1; % will contains cartography points status
found=false;
resign=false;
origin=false;
%if (carto(targetX,targetY)>maxCartoWeight || carto(currentX,currentY)>maxCartoWeight )  % origine or target not reachable
if (QueryCartoAvailability(currentX,currentY,currentHeadingGrad,cartoId,1) == 0 || QueryCartoAvailability(targetX,targetY,targetHeadingGrad,cartoId,1) == 0 )  % origine or target not reachable
	cost=-1
	resign=true
	return
endif

	if (currentHeading<=90)
	%{
		if ((QueryCartoAvailability(currentX,currentY,floor(currentHeading/2),cartoId,1)) == 1 && (QueryCartoAvailability(currentX,currentY,0,cartoId,1) == 1))
			startHeading=0;
		endif
	%}
		startHeading=0;
		forward=1;
	endif
	if (currentHeading>=270)
		%{
		if ((QueryCartoAvailability(currentX,currentY,floor(currentHeading +(360-currentHeading)/2),cartoId,1)) == 1 && (QueryCartoAvailability(currentX,currentY,0,cartoId,1) == 1))
			startHeading=0;

		endif
			%}
		startHeading=0;
		forward=1;
	endif
	if (currentHeading>90 && currentHeading<=270)
		%{
		if ((QueryCartoAvailability(currentX,currentY,floor(currentHeading+(180-currentHeading)/2),cartoId,1)) == 1 && (QueryCartoAvailability(currentX,currentY,180,cartoId,1) == 1))
			startHeading=180;

		endif
			%}
		startHeading=180;
		forward=-1;
	endif
if (startHeading==-1)
	cost=-3;
	return
else
%	AStartHeading=currentHeadingGrad;
endif
%}
if (targetX==currentX && targetY==currentY)       % target already reached
	cost=0;
	return
endif
while (found == false && resign == false)
	if (size(openList,1)==0)         % not more path to expand
		resign=true;
		cost=-2;
		idx=-1;
		path=[];
		return
	else
%		openList
		[minV,idx]=min(openList(:,6));  % look for the lowest value of (cost+heuristic)
		g=openList(idx,1)+1;            % get the step number
		prevX=openList(idx,2);
		prevY=openList(idx,3);
		prevCost=openList(idx,7);
		prevAction=openList(idx,4:5);
%		printf("minV:%d step number:%d min X:%d min Y:%d prevCost:%d prevAction:( %d %d ). .\n",minV,g,prevX,prevY,prevCost,prevAction(1) ,prevAction(2))
		for i=1:nbActions
			if (prevX==targetX && prevY==targetY)
				found=true;
				cost=openList(idx,7);
				a=targetX;
				b=targetY;
				AStarStep=[a,b];
				ss=actions==prevAction;
				actionSteps(a,b)=find(ss(:,1).*ss(:,2));
				while (origin==false)
					AStarPath=[AStarPath,[(actionSteps(a,b))]];
					as=a;
%					actionSteps(a,b);
					a=a-actions(actionSteps(a,b),1);
					b=b-actions(actionSteps(as,b),2);
					AStarStep=[AStarStep;[a,b]];
					if ([a,b]==initPos(1:2))
						origin=true;
						AStarPath=fliplr(AStarPath);
					endif
				end
				if (plotOn)
					AStarShowStep(AStarStep);
				endif
				return
			else
				AStarHeading = mod(AStarHeading + actionsRotation(i),2*pi());
%				newX=floor(prevX+actions(i,1)+stepSize*cos(AStarHeading));
%				newY=floor(prevY+actions(i,2)+stepSize*sin(AStarHeading));
				newX=floor(prevX+actions(i,1));
				newY=floor(prevY+actions(i,2));
				if (newX >=1 && newY >=1 && newX <=size(carto,1) && newY <=size(carto,2))
					cartoWeight=carto(newX,newY);

% check space available for the entire robot taking into account the orientation
%					if (QueryCartoAvailability(newX,newY,mod(360+AStarHeading*180/pi(),360),cartoId,0)==1)
					if (QueryCartoAvailability(newX,newY,AStarHeading,cartoId,0)==1)
%					if (QueryCartoAvailability(newX,newY,5,cartoId,0)==1)
%					if (carto1(newX,newY)<=20)
						if (expandList(newX,newY)==-1 && closed(newX,newY)==0)
%							printf("new X:%d new Y:%d . \n",newX,newY)
							newCost=prevCost+AStarActionCost(prevAction,actions(i,:),stepSize)+weightCarto*cartoWeight;
							openList=[openList;[g,newX,newY,[actions(i,:)],newCost+AStarHeuristic(newX,newY,targetX,targetY)],newCost,AStarHeading]; 
%							ss=actions==actions(i,:);
%							actionSteps(newX,newY)=find(ss(:,1).*ss(:,2));	
							actionSteps(newX,newY)=i;						
							closed(newX,newY)=1;
						endif
					else
%						QueryCartoAvailability(newX,newY,AStarHeading,cartoId,1);
						expandList(newX,newY)=0;
						AStarHeading=currentHeadingGrad;
					endif
				endif
			endif
		endfor
		expandList(openList(idx,2),openList(idx,3))=openList(idx,1);
		closed(openList(idx,2),openList(idx,3))=1;
		currentCost=g;
		openList(idx,:)=[];
	endif
end

endfunction