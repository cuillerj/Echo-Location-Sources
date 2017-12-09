function [apRobot,robot] = testMoveAcrossNarrowPath(apRobot,robot,pathDistance,reqMove);
 width=78;
 length=80;
 echoToGet=200;
 northHeading=74;
 startToEntryDistance=80;
 robotStatus=robot.runningStatus;
 debugOn=1;
 if (robotStatus<=0)
          printf(mfilename);
          printf(" wait for robot to be ready  *** ");
          printf(ctime(time()));
  endif
  while (robotStatus<=0)              % wait for robot to be ready
          robotStatus=robot.runningStatus;
          pause(2);
          printf(".");
  end
 pause(1);
robot.MoveAcrossNarrowPass(pathDistance,width,length,startToEntryDistance,northHeading,reqMove,echoToGet);
apRobot = setfield(apRobot,"waitFor",robot.moveAcrossPassEnded);
[apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
robot.RequestNarrowPathMesurments;
pause(2);
robot.RequestNarrowPathEchos;