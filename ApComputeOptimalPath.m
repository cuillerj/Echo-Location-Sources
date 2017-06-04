function [apRobot,robot,next,forward] = ApComputeOptimalPath(apRobot,robot,targetL,plotOn)
  forward=1;
  cost=0;
  currentL=apGet(apRobot,"location");
  apRobot = setfield(apRobot,"newTarget",false);
  pathStep=[];
  optimalPath=apGet(apRobot,"optimalPath");
  carto=apGet(apRobot,"carto");
  stepSize=apGet(apRobot,"stepSize");
  [apRobot,robot,closestStartOptimalPoint,startIdx] = ApFindClosestOptimalLocation(apRobot,robot,currentL);
  [apRobot,robot,closestEndOptimalPoint,endIdx] = ApFindClosestOptimalLocation(apRobot,robot,targetL);
  printf(mfilename);
  printf(" current loc: %d %d %d optimal start %d %d %d optimal end %d %d %d target %d %d %d *** ",currentL(1),currentL(2),currentL(3),closestStartOptimalPoint(1),closestStartOptimalPoint(2),closestStartOptimalPoint(3),closestEndOptimalPoint(1),closestEndOptimalPoint(2),closestEndOptimalPoint(3),targetL(1),targetL(2),targetL(3));
 	printf(ctime(time())); 
  if (sum(currentL(1:2)==closestStartOptimalPoint(1:2))!=2)  % (x,y) are different
     printf("1:");
  	 [rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,currentL,[closestStartOptimalPoint(1),closestStartOptimalPoint(2),closestStartOptimalPoint(3)]);
     if (possible)
         printf(mfilename);
         printf(" call astar search to reach the optimal path *** ");
 	       printf(ctime(time())); 
       pathStep=[pathStep;[closestStartOptimalPoint(1),closestStartOptimalPoint(2)]]
     else
       [apRobot,robot,AStarPath,pathStep,cost,forward] = ApAStarSearch(apRobot,robot,currentL,closestStartOptimalPoint,plotOn);
     endif
  endif
  if (endIdx-startIdx>2)
    for i=startIdx+1:endIdx-1
      if (i==endIdx-1)
        pathStep=[pathStep;[optimalPath(i,1),optimalPath(i,2)]];      
      else
        if (optimalPath(i+1,3)!=optimalPath(i,3))
          pathStep=[pathStep;[optimalPath(i,1),optimalPath(i,2)]];
         endif
      endif
    end
  endif
  if (sum(targetL(1:2)==closestEndOptimalPoint(1:2))!=2)
      printf("2:");
     [rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,[closestEndOptimalPoint(1),closestEndOptimalPoint(2),closestEndOptimalPoint(3)],targetL);
     if (possible)
         printf(mfilename);
         printf(" call astar search to leave the optimal path *** ");
 	       printf(ctime(time())); 
       pathStep=[pathStep;[closestEndOptimalPoint(1),closestEndOptimalPoint(2)]];
     else
      [apRobot,robot,AStarPath,pathStep,cost,forward] = ApAStarSearch(apRobot,robot,closestEndOptimalPoint,targetL,plotOn);
     endif     

  endif
       pathStep=[pathStep;[targetL(1),targetL(2)]];
       apRobot = setfield(apRobot,"pathStep",pathStep);
       next=pathStep(1,1:2)
  endfunction
  