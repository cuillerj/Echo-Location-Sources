function [apRobot,robot,AStarPath,AStarStep,cost] = ApAStarSearch(apRobot,robot,currentL,targetL,plotOn)
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
 carto=apGet(apRobot,"carto");
 targetX=targetL(1);
 targetY=targetL(2);
 targetHeading=targetL(3);
 currentHeading=mod(currentL(3)+360,360);
 direction=[targetX-currentL(1),targetY-currentL(2)];
 currentHeadingGrad=currentHeading*pi()/180;
 vectHeading=[cos(currentHeadingGrad),sin(currentHeadingGrad)];
 projectionHeading=direction*vectHeading';
 %forward=0;

 weightCarto=20;     % weight used to balance way depending on cartography weight and action cost
  targetHeadingGrad=targetHeading*pi()/180;
  AStarPath=[];
  AStarStep=[];
  stepSize=apGet(apRobot,"stepSize");
  currentX=stepSize*(floor(currentL(1)/stepSize))+floor(stepSize/2); % adjust position to center of square carto
  currentY=stepSize*(floor(currentL(2)/stepSize))+floor(stepSize/2);
  targetX=stepSize*(floor(targetX/stepSize))+floor(stepSize/2);
  targetY=stepSize*(floor(targetY/stepSize))+floor(stepSize/2);
  angleOneHole=apGet(apRobot,"angleOneHole");
  distanceOneHole=apGet(apRobot,"distanceOneHole");

[nbX,nbY]=size(carto); % a nb of X positions b nb of Y positions
initPos=[currentL(1),currentL(2)];
pos=initPos;
targetPos=[targetX,targetY];
%maxCartoWeight=20; % threshold over that means space (x,y) is not available
%weightCarto=20;     % weight used to balance way depending on cartography weight and action cost

actions=[[stepSize,0];[-stepSize,0];[0,stepSize];[0,-stepSize];[stepSize,stepSize];[stepSize,-stepSize];[-stepSize,stepSize];[-stepSize,-stepSize]];
    
shiftRotation=[0,4,2,6,1,7,3,5].*(pi()/4);     
  
%AStarHeading=currentHeadingGrad

nbActions=size(actions,1);
mailleRotation=nbActions;
deltaMailleRotation=360/mailleRotation;
currentHeadingMaille=floor(currentHeading/deltaMailleRotation);
currentHeadingGrad= currentHeadingMaille*2*pi()/mailleRotation;
currentH=currentHeadingGrad;
actionsRotation=mod((shiftRotation).-currentHeadingGrad,2*pi());   
currentAction=[0,0];
currentCost=0;
%{ openList contains (X,Y) points that are to be developped
%}
AStarPath=[];         
AStarStep=[];
cost=-1;
closeSet=[];
openSet=[pos(1),pos(2),currentHeadingMaille+1];
cameFrom=zeros(nbX,nbY,mailleRotation);
cameWith=zeros(nbX,nbY,mailleRotation);
gScore=ones(nbX,nbY,mailleRotation).*Inf;
gScore(pos(1),pos(2),currentHeadingMaille+1)=0;
fScore=ones(nbX,nbY,mailleRotation).*Inf;
fScore(pos(1),pos(2),currentHeadingMaille+1)=AStarHeuristic(pos(1),pos(2),targetX,targetY);
found=false;
current=[pos(1),pos(2),currentHeadingMaille+1];
while (size(openSet,1)!=0 || found==true)
	[valueMin,idxMin]=min(fScore(:));
	[currentX,currentY,currentH]=ind2sub(size(fScore),idxMin);
	if ([currentX,currentY]==[targetX,targetY])
		found=true;
%		save  ("cameFrom.mat","cameFrom")
		cost=fScore(currentX,currentY,currentH);
		AStarStep=[AStarPath;[currentX,currentY]];
		a=targetX;
		b=targetY;
	%		AStarStep=[AStarStep;[currentH]];
		backEnd=false;

		while (backEnd!=true)
			actionBack=cameFrom(currentX,currentY,currentH);
			backX=currentX-actions(actionBack,1);
			backY=currentY-actions(actionBack,2);

			AStarStep=[AStarStep;[backX,backY]];         
			AStarPath=[AStarPath;[actionBack]];
			if (backX == pos(1) && backY == pos(2))
				backEnd=true;
			else
				currentH=cameWith(currentX,currentY,currentH);
				currentX=backX;
				currentY=backY;
%				currentH=backHeadingMaille+1;
%				backHeadingMaille=floor((backH-1)*180/pi()/deltaMailleRotation)+1
			endif
		end
		AStarPath=flipud(AStarPath);
		AStarStep=flipud(AStarStep);
		if (plotOn)
			ApAStarShowStep(apRobot,AStarStep,"AStar search result");
			hold off
		endif
    %{
		if (projectionHeading>=0)
			forward=1;
		else
			forward=-1;
		endif
    %}
		printf(mfilename);
		printf(" *** cpu search:%f . ",cputime()-cpu1);
		printf(ctime(time()))
		return
	else
		closeSet=[[closeSet];[currentX,currentY,currentH]];
		idxOpenSet=find(ismember(openSet,[currentX,currentY,currentH],'rows'));
		openSet(idxOpenSet,:)=[];
		fScore(currentX,currentY,currentH)=[Inf];
		for i=1:nbActions
				neighborX=floor(currentX+actions(i,1));
				neighborY=floor(currentY+actions(i,2));
%				neighborH=mod(shiftRotation(i)-shiftRotation(currentH),2*pi());
				neighborH=mod(shiftRotation(i)-(currentH-1)*2*pi()/mailleRotation,2*pi());
				neighborHeadingMaille=floor(neighborH*180/pi()/deltaMailleRotation);
				neighbor=[neighborX,neighborY,neighborHeadingMaille+1];
				if (neighborX >=1 && neighborY>=1 && neighborX <=size(carto,1) && neighborY <=size(carto,2) && ApQueryCartoAvailability(apRobot,[neighborX,neighborY,neighborH],1,0)==1)
					if (sum(ismember(closeSet,neighbor,'rows'))==0)
%						tentative_gScore = gScore(currentX,currentY,currentH) + sqrt(actions(i,1)^2+actions(i,2)^2);
						if (cameFrom(currentX,currentY,currentH)!=0)
%actions(cameFrom(currentX,currentY,currentH),:)
%actions(i,:)
							neibghboorCost=AStarActionCost(actions(cameFrom(currentX,currentY,currentH),:),actions(i,:),stepSize);
						else
							neibghboorCost=sqrt(actions(i,1)^2+actions(i,2)^2);
						endif
						tentative_gScore = gScore(currentX,currentY,currentH) + neibghboorCost;
						if (sum(ismember(openSet,neighbor,'rows'))==0)
							openSet=[[openSet];[neighbor]];
							if (tentative_gScore < gScore(neighborX,neighborY,neighborHeadingMaille+1))
								cameFrom(neighborX,neighborY,neighborHeadingMaille+1) = i;
								cameWith(neighborX,neighborY,neighborHeadingMaille+1) = currentH;
								gScore(neighborX,neighborY,neighborHeadingMaille+1) = tentative_gScore;
								cartoWeight=weightCarto*carto(neighborX,neighborY);
								fScore(neighborX,neighborY,neighborHeadingMaille+1) = gScore(neighborX,neighborY,neighborHeadingMaille+1) + AStarHeuristic(neighbor(1),neighbor(2),targetX,targetY)+cartoWeight;
							endif
						endif
					endif
				endif
		endfor
	endif
		current=[currentX,currentY,currentH];
end
endfunction