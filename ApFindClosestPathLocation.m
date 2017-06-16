function [apRobot,robot,closestPathPoint,closestIdx] = ApFindClosestPathLocation(apRobot,robot,location)
%{ 
 find the closest points located on the optimal path
%}
  closePathDistance=apGet(apRobot,"closePathDistance");
  farPointDistance=apGet(apRobot,"farPointDistance");
  pathStep=apGet(apRobot,"pathStep");
  [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,location,apGet(apRobot,"destination"),0);
  if (possible && distance <= farPointDistance)  % is it possible to go straigt to the target
      closestPathPoint=[pathStep(size(pathStep,1),1),pathStep(size(pathStep,1),2)];
      closestIdx=size(pathStep,1);
  else
      currentX=location(1);
      currentY=location(2);
      prevDist=inf;
      closestIdx=1;
      for i=1:size(pathStep,1)  % look for the a step close enough to the current poistion
        dist=sqrt((pathStep(i,1)-currentX)^2+(pathStep(i,2)-currentY)^2);
        if (dist<closePathDistance)
          closestIdx=i;
          i=size(pathStep,1)+1;
        endif
      end
      newPath=[];
      for i=closestIdx+1:size(pathStep,1)   % delete previous steps
        newPath=[newPath;[pathStep(i,:)]];
      end
      apRobot = setfield(apRobot,"pathStep",newPath);
      closestIdx=1;
      for i=1:size(newPath,1)
        dist=sqrt((newPath(i,1)-currentX)^2+(newPath(i,2)-currentY)^2);
        if (dist<=prevDist && dist>=closePathDistance)
            [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,location,newPath(i,:),0);
            if (possible)
              prevDist=dist;
              closestIdx=i;
            endif
        endif
      endfor
      closestPathPoint=[newPath(closestIdx,1),newPath(closestIdx,2)];
  endif
	printf(mfilename);
	printf(" *** closest step is: X:%d Y:%d H:%d. *** ", closestPathPoint(1),closestPathPoint(2));
	printf(ctime(time()));

endfunction