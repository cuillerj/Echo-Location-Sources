function [apRobot] = ApClearPathStep(apRobot)
    closePathDistance=apGet(apRobot,"closePathDistance");
    farPointDistance=apGet(apRobot,"farPointDistance");
    pathStep=apGet(apRobot,"pathStep");
    location=apGet(apRobot,"location");
    currentX=location(1);
    currentY=location(2);
    updatedPath=[];
    i=1;
    while(pathStep(i,3)!=0 && i<size(pathStep,1))   % step already tried
          if (sqrt((pathStep(i,1)-currentX)^2+(pathStep(i,2)-currentY)^2)<=closePathDistance)
            % skip step
            i=i+1;
          else
            break;
          endif
    end
    idx=i;
    if (i<size(pathStep,1))
      [apRobot,rotation,distance,possible]=ApCheckStraightMovePossibility(apRobot,[currentX,currentY,location(3)],[pathStep(i+1,1),pathStep(i+1,2),location(3)],0);
      if (possible && sqrt((pathStep(i+1,1)-currentX)^2+(pathStep(i+1,2)-currentY)^2)<=farPointDistance)
        idx=i+1;
      endif
    endif
    printf(mfilename);
    printf(" *** remove:%d steps from path *** ",idx-1);
 	  printf(ctime(time()))
    for (i=idx:size(pathStep,1))
       updatedPath=[updatedPath;[pathStep(i,:)]];
    end
    apRobot = setfield(apRobot,"pathStep",updatedPath);
  endfunction