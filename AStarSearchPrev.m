function [AStarPath,AStarStep,cost,startHeading,forward] = AStarSearch(carto,currentX,currentY,currentHeading,targetX,targetY,targetHeading,plotOn)
%{ 
 AStarPath will return the ordered list of actions to go from current to target position
 AStarStep will return the list of positions touched to go from current to target position(ordered from target position to  current)
		step will be ploted if plotOn is true
 cost will return cost to go from current to target position

  printf("astar search\n")
 %}
 cpu1=cputime();
 iterCount=0;
 print=false;   % set true for debug
 debug=false;
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
%cartoId=1;        %  multiple cartography to be supported later
%load carto1;       % cartographie matrix (1x1 cm)
if (exist("carto")==false)
	load carto1;
	printf("load carto *** ")
	printf(ctime(time()))
	carto=carto1;
endif
[nbX,nbY]=size(carto); % a nb of X positions b nb of Y positions
initPos=[currentX,currentY];
pos=initPos;
targetPos=[targetX,targetY];
%maxCartoWeight=20; % threshold over that means space (x,y) is not available
weightCarto=20;     % weight used to balance way depending on cartography weight and action cost

actions=[
		[stepSize,0];[-stepSize,0];
		[0,stepSize];[0,-stepSize];
		[stepSize,stepSize]; [stepSize,-stepSize];  % actions list (deltaX,deltaY)
		[-stepSize,stepSize];[-stepSize,-stepSize];
		];
actionsRotation=[0,pi(),pi()/2,3*pi()/2,pi()/4,7*pi()/4,pi()*3/4,pi()*5/4];
AStarHeading=currentHeadingGrad;
nbActions=size(actions,1);
currentAction=[0,0];
g=0;
currentCost=0;
%{ openList contains (X,Y) points that are to be developped
%}
openList=[g,pos(1),pos(2),currentAction,currentCost+AStarHeuristic(currentX,currentY,targetX,targetY),currentCost,0];
AStarPath=[];         
AStarStep=[];
%actionSteps=zeros(nbX,nbY);  % will keep for each cartography point the last action to go there
[c,deltaHeading]=size(actionsRotation);
closed=zeros(nbX,nbY,deltaHeading);       % will contains cartography points status
expandList=ones(nbX,nbY,deltaHeading).*-1; % will contains cartography points status
actionSteps=zeros(nbX,nbY,deltaHeading);  % will keep for each cartography point the last action to go there
found=false;
resign=false;
origin=false;
%if (carto(targetX,targetY)>maxCartoWeight || carto(currentX,currentY)>maxCartoWeight )  % origine or target not reachable
if (QueryCartoAvailability(carto,currentX,currentY,currentHeadingGrad,debug) == 0 || QueryCartoAvailability(carto,targetX,targetY,targetHeadingGrad,debug) == 0 )  % origine or target not reachable
	cost=-1
	resign=true
	return
endif

	if (currentHeading<=90)
		startHeading=0;
		forward=1;
	endif
	if (currentHeading>=270)
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
AStarHeading=0;
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
		prevI=openList(idx,8);
%		printf("minV:%f step number:%d min X:%d min Y:%d prevCost:%d prevAction:( %d %d )  prevI:%d .\n",minV,g,prevX,prevY,prevCost,prevAction(1) ,prevAction(2),prevI)
		for i=1:nbActions
			iterCount++;
			if (prevX==targetX && prevY==targetY)
				printf("iteration count:%d *** ",iterCount);
				printf(ctime(time()));
				found=true;
				cost=openList(idx,7);
				a=targetX;
				b=targetY;
				AStarStep=[a,b];
				c=prevI;
				AStarPath=[c];
				if (debug==true)
					printf("found prevI %d . *** ",c)
					printf(ctime(time()))
					save ("actionSteps.mat","actionSteps")
					save ("openList.mat","openList")
					save ("expandList.mat","expandList")
					save ("closed.mat","closed")
				endif
				while (origin==false)		
					a1=a-actions(c,1);
					b1=b-actions(c,2);
					c=actionSteps(a,b,c);
					if (c==0)
						AStarStep=[AStarStep;[a,b]];
						AStarStep=[AStarStep;[initPos(1:2)]];
						AStarPath=fliplr(AStarPath);
						origin=true;
					else
						AStarStep=[AStarStep;[a,b]];
						AStarPath=[AStarPath,[c]];
						a=a1;
						b=b1;				
						endif
				end
				if (plotOn)
					AStarShowStep(AStarStep);
					hold off
				endif
				printf("cpu search:%f . ",cputime()-cpu1);
				printf(ctime(time()))
				return
			else
				AStarHeading = mod(actionsRotation(i)+currentHeadingGrad,2*pi());  % ajout +currentHeadingGrad le 20/09/2016
				newX=floor(prevX+actions(i,1));
				newY=floor(prevY+actions(i,2));
				if (newX >=1 && newY >=1 && newX <=size(carto,1) && newY <=size(carto,2) && QueryCartoAvailability(carto,newX,newY,AStarHeading,0)==1)
					cartoWeight=carto(newX,newY);				
						if (expandList(newX,newY,i)==-1 && closed(newX,newY,i)==0)
							newCost=prevCost+AStarActionCost(prevAction,actions(i,:),stepSize)+weightCarto*cartoWeight;
							openList=[openList;[g,newX,newY,[actions(i,:)],newCost+AStarHeuristic(newX,newY,targetX,targetY)],newCost,i]; 
							actionSteps(newX,newY,i)=prevI;								
							expandList(newX,newY,i)=0;
						endif
					else
						closed(newX,newY,i)=1;
				endif
			endif
		endfor
%		expandList(newX,newY,[])=0;
%		closed(newX,newY,[])=1;
		currentCost=g;
		openList(idx,:)=[];
	endif
end
endfunction