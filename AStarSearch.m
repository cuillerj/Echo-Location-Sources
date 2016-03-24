function [AStarPath,AStarStep,cost] = AStarSearch(currentX,currentY,targetX,targetY,plotOn)
%{ 
 AStarPath will return the ordered list of actions to go from current to target position
 AStarStep will return the list of positions touched to go from current to target position(ordered from target position to  current)
		step will be ploted if plotOn is true
 cost will return cost to go from current to target position
  %}
  stepSize=10;       % n lenght of the first kind of action msut match with carto square size
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
maxCartoWeight=10; % threshold over that means space (x,y) is not available
weightCarto=5;     % weight used to balance way depending on cartography weight and action cost

actions=[
		[stepSize,0];[-stepSize,0];
		[0,stepSize];[0,-stepSize];
		[stepSize,stepSize]; [stepSize,-stepSize];  % actions list (deltaX,deltaY)
		[-stepSize,stepSize];[-stepSize,-stepSize];
		];

nbActions=size(actions,1);
currentAction=[0,0];
g=0;
currentCost=0;
%{ openList contains (X,Y) points that are to be developped
%}
openList=[g,pos(1),pos(2),currentAction,currentCost+AStarHeuristic(currentX,currentY,targetX,targetY),currentCost];
AStarPath=[];         
AStarStep=[];         
actionSteps=zeros(a,b);  % will keep for each cartography point the last action to go there
closed=zeros(a,b);       % will contains cartography points status
expandList=ones(a,b).*-1; % will contains cartography points status
found=false;
resign=false;
origin=false;
if (carto(targetX,targetY)>maxCartoWeight || carto(currentX,currentY)>maxCartoWeight )  % origine or target not reachable
	cost=-1;
	return
endif
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
		[minV,idx]=min(openList(:,6));  % look for the lowest value of (cost+heuristic)
		g=openList(idx,1)+1;            % get the step number
		prevX=openList(idx,2);
		prevY=openList(idx,3);
		prevCost=openList(idx,7);
		prevAction=openList(idx,4:5);
%		printf("nin X:%d min Y:%d . \n",prevX,prevY)
		for i=1:nbActions
			if (prevX==targetX && prevY==targetY)
				found=true;
				cost=openList(idx,7);
				a=targetX;
				b=targetY;
				AStarStep=[a,b];
				actionSteps(a,b)=i;
				while (origin==false)
					AStarPath=[AStarPath,[(actionSteps(a,b))]];
					actionSteps(a,b);
					as=a;
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
				newX=prevX+actions(i,1);
				newY=prevY+actions(i,2);
				
				if (newX >=1 && newY >=1 && newX <=size(carto,1) && newY <=size(carto,2))
					cartoWeight=carto(newX,newY);
					if (cartoWeight<=maxCartoWeight)
						if (expandList(newX,newY)==-1 && closed(newX,newY)==0)
%							printf("new X:%d new Y:%d . \n",newX,newY)
							newCost=prevCost+AStarActionCost(prevAction,actions(i,:),stepSize)+weightCarto*cartoWeight;
							openList=[openList;[g,newX,newY,[actions(i,:)],newCost+AStarHeuristic(newX,newY,targetX,targetY)],newCost]; 
							actionSteps(newX,newY)=i;
							closed(newX,newY)=1;
						endif
					else
						expandList(newX,newY)=0;
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