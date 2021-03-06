function [apRobot,robot,closestPathPoint,closestIdx,forward] = ApFindClosestPathLocation(apRobot,robot,location)
%{ 
 find the closest points located on the optimal path
%}
  closePathDistance=apGet(apRobot,"closePathDistance");
  farPointDistance=apGet(apRobot,"farPointDistance");
  pathStep=apGet(apRobot,"pathStep");
  optimalPath=apGet(apRobot,"optimalPath");
  forward=1;
  [apRobot,robot,closestOptimalPoint,startIdx] = ApFindClosestOptimalLocation(apRobot,robot,apGet(apRobot,"location"));
  [apRobot,robot,closestOptimalPoint,endIdx] = ApFindClosestOptimalLocation(apRobot,robot,apGet(apRobot,"destination"));
  newInstruction=false;
  for idx=startIdx:endIdx-1
    if (optimalPath(idx,4)!=optimalPath(idx+1,4))  % is there a new path instruction to apply between current and target position
      newInstruction=true;
      printf(mfilename);
      printf(" path instruction to proceed *** ")
 	    printf(ctime(time())); 
      idx=endIdx;
     endif
  end
  possible=false;
  if (!newInstruction)
    [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,location,apGet(apRobot,"destination"),0);
  endif
  if (possible && distance <= farPointDistance)  % is it possible to go straigt to the target
      closestPathPoint=[pathStep(size(pathStep,1),1),pathStep(size(pathStep,1),2)];
      closestIdx=size(pathStep,1);
  else
      currentX=location(1);
      currentY=location(2);
      prevDist=inf;
      closestIdx=0;
      i=0;
      while(i<(size(pathStep,1)-1))
          i=i+1;
          [apRobot,rotation,distance,possible]=ApCheckStraightMovePossibility(apRobot,[currentX,currentY,location(3)],[pathStep(i,1),pathStep(i,2),location(3)],0);    
          if (possible)
               dist=sqrt((pathStep(i,1)-currentX)^2+(pathStep(i,2)-currentY)^2);
                if (dist<prevDist)
                  prevDist=dist;
                  closestIdx=i;
                else
                  closestIdx=i-1;
                  break
                endif    
          endif
       
      end
      printf(mfilename);
      if (closestIdx>0)
        pathStep(closestIdx,3)=pathStep(closestIdx,3)+1;
        if(sqrt((pathStep(closestIdx+1,1)-currentX)^2+(pathStep(closestIdx+1,2)-currentY)^2)<farPointDistance+closePathDistance)  % too close to this step
          [apRobot,rotation,distance,possible]=ApCheckStraightMovePossibility(apRobot,[currentX,currentY,location(3)],[pathStep(closestIdx+1,1),pathStep(closestIdx+1,2),location(3)],0);    
          if (possible)         % if possible skip one step
            closestIdx=closestIdx+1;
            pathStep(closestIdx,3)=pathStep(closestIdx,3)+1;
          endif
        endif
        closestPathPoint=[pathStep(closestIdx,1),pathStep(closestIdx,2)];
        forward=pathStep(closestIdx,4);
        printf(" *** closest step is: X:%d Y:%d idx:%d forward:%d*** ", closestPathPoint(1),closestPathPoint(2),closestIdx,forward);
       else 
         closestPathPoint=[,];
         closestIdx=-1;
         printf(" *** closest step not found *** ");
       endif
       	printf(ctime(time()));
  endif




endfunction