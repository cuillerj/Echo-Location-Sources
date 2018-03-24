function [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,targetL,plotOn)
  %{
  this function compute the optimal path steps from apRobot current position to the target and create the pathStep attribute of apRobot
    find the closest starting point to the current position of the optimal path 
    find the closest ending point to the target position of the optimal path 
    add steps from the starting point to the ending point
    pathStep for each step = (posX , posY, 0 (step not reached), step intruction)
  %}
  retCode=-1; 

  cost=0;
  currentL=apGet(apRobot,"location");
  apRobot = setfield(apRobot,"newTarget",false);
  pathStep=[];
  optimalPath=apGet(apRobot,"optimalPath");
  carto=apGet(apRobot,"carto");
  stepSize=apGet(apRobot,"stepSize");
  farPointDistance=apGet(apRobot,"farPointDistance");
  [apRobot,robot,closestStartOptimalPoint,startIdx] = ApFindClosestOptimalLocation(apRobot,robot,currentL);
  [apRobot,robot,closestEndOptimalPoint,endIdx] = ApFindClosestOptimalLocation(apRobot,robot,targetL);
  printf(mfilename);
  printf(" current loc: %d %d %d optimal start %d %d %d optimal end %d %d %d target %d %d %d *** ",currentL(1),currentL(2),currentL(3),closestStartOptimalPoint(1),closestStartOptimalPoint(2),closestStartOptimalPoint(3),closestEndOptimalPoint(1),closestEndOptimalPoint(2),closestEndOptimalPoint(3),targetL(1),targetL(2),targetL(3));
 	printf(ctime(time())); 
  if (startIdx>endIdx)                  % means we are going backward regarding path
    idx=size(optimalPath)(1)-endIdx;   % reverse path and idx
    startIdx=size(optimalPath)(1)-startIdx;
    endIdx=idx;
    optimalPath=flipud(optimalPath);
    optimalPath(:,3)=mod(optimalPath(:,3)+180,360);
  endif
  if (sum(currentL(1:2)==closestStartOptimalPoint(1:2))!=2)  % (x,y) are different
  	 [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,currentL,[closestStartOptimalPoint(1),closestStartOptimalPoint(2)]);
     if (possible)
       pathStep=[pathStep;[closestStartOptimalPoint(1),closestStartOptimalPoint(2),0,closestStartOptimalPoint(4)]];
     else
       printf(mfilename);
       printf(" call astar search to reach the optimal path *** ");
 	     printf(ctime(time())); 
       [apRobot,robot,AStarPath,pathStep,cost,forward] = ApAStarSearch(apRobot,robot,currentL,closestStartOptimalPoint,plotOn);
     endif
  endif
  lastStep=[closestStartOptimalPoint(1),closestStartOptimalPoint(2),0,closestStartOptimalPoint(4)];

  if (abs(endIdx-startIdx)>2)
    for i=startIdx+1:endIdx-1
      if (i==endIdx-1)
        pathStep=[pathStep;[optimalPath(i,1),optimalPath(i,2),0,optimalPath(i,4)]];      
      else
        if (optimalPath(i+1,3)!=optimalPath(i,3))
          pathStep=[pathStep;[optimalPath(i,1),optimalPath(i,2),0,optimalPath(i,4)]];
          lastStep=[optimalPath(i,1),optimalPath(i,2),0,optimalPath(i,4)];
         endif
         if (sqrt((lastStep(1)-optimalPath(i,1))^2+(lastStep(2)-optimalPath(i,2))^2)>=farPointDistance-1)
           pathStep=[pathStep;[optimalPath(i,1),optimalPath(i,2),0,optimalPath(i,4)]];
           lastStep=[optimalPath(i,1),optimalPath(i,2),0,optimalPath(i,4)];
          endif
      endif
    end
  endif
  if (sum(targetL(1:2)==closestEndOptimalPoint(1:2))!=2)
     [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,[closestEndOptimalPoint(1),closestEndOptimalPoint(2),0,closestEndOptimalPoint(4)],targetL);
     if (possible)
       pathStep=[pathStep;[closestEndOptimalPoint(1),closestEndOptimalPoint(2),0,closestEndOptimalPoint(4)]];
     else
       printf(mfilename);
       printf(" call astar search to leave the optimal path *** ");
 	     printf(ctime(time()));      
      [apRobot,robot,AStarPath,pathStep,cost,forward] = ApAStarSearch(apRobot,robot,closestEndOptimalPoint,targetL,plotOn);
     endif     
       printf(mfilename);
       printf("  path= ");
       for i=1:size(pathStep,1)      
          printf(" (%d,%d) ",pathStep(i,1),pathStep(i,2));
       end
 	     printf(ctime(time())); 
  endif
       pathStep=[pathStep;[targetL(1),targetL(2),0,0]];
 %      next=pathStep(1,1:2);
    %   pathStep(1,3)=pathStep(1,3)+1;
       retCode=0;      
       apRobot = setfield(apRobot,"pathStep",pathStep);
       if (plotOn)
          ApShowStep(apRobot,pathStep(:,1:2),"optimal path",1);
       endif
 %      pathStep
  endfunction
  