function [targetReached] = ApTargetReached(apRobot,robot);
  closeTargetDistance=apGet(apRobot,"closeTargetDistance");
  location=apGet(apRobot,"location");
  destination=apGet(apRobot,"destination");
  remainingDistance=round(sqrt((location(1)-destination(1))^2+(location(2)-destination(2))^2));
  printf(mfilename);
  printf(" Distance to the target:%d *** ",remainingDistance);
  printf(ctime(time()))
  if(remainingDistance<=closeTargetDistance)
    targetReached=true;
  else
    targetReached=false;
  endif
 endfunction