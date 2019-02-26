function [apRobot,robot] = testMoveAcrossNarrowPath(apRobot,robot,pathDistance,reqMove);
 width=78;
 length=120;
 echoToGet=100;
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
 pause(10);
robot.MoveAcrossNarrowPass(pathDistance,width,length,startToEntryDistance,northHeading,reqMove,echoToGet);
apRobot = setfield(apRobot,"waitFor",robot.moveAcrossPassEnded);
[apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
pause(2);
robot.RequestNarrowPathMesurments;
pause(2);
robot.RequestNarrowPathEchos;
pause(2);
robot.GetSubsystemLocation();
printf("distance to pass start:%dcm to pass entry:%dcm  path lengh:%dcm  after distance:%dcm \n",robot.GetPathDistances(1),robot.GetPathDistances(2),robot.GetPathDistances(3),robot.GetPathDistances(4));
endfunction