function [apRobot,robot] = testMoveAcrossNarrowPath(apRobot,robot,pathDistance,reqMove);
 width=76;
 length=32;
 echoToGet=100;
 northHeading=74;
 startToEntryDistance=60;
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
debugOn;
[apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
 if (retCode!=0)
        [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
        if (issue)
            printf("issue \n");
            return
        endif
 endif
if (retCode!=0)
    if(retCode==robot.moveAcrossPathKoKoDueToWheelStopped)
          robot.MoveAcrossNarrowPass(pathDistance,width,length,startToEntryDistance/2,northHeading,reqMove,echoToGet);
          apRobot = setfield(apRobot,"waitFor",robot.moveAcrossPassEnded);
          [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
           if (retCode!=0)
                  [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
                  if (issue)
                      printf("issue \n");
                      return
                  endif
           endif
      endif
     printf("retCode:%d \n",retCode);
else
    robot.GetSubsystemLocation();
    pause(3);
    robot.RequestNarrowPathMesurments;
    pause(3);
    retry=0;
     while (robot.pendingNarrowPathMesurments && retry <5)
              robot.RequestNarrowPathMesurments;
              retry++;
              pause(3);
     endwhile
     pathDistances=[robot.GetPathDistances(1),robot.GetPathDistances(2),robot.GetPathDistances(3),robot.GetPathDistances(4)];
     printf("distance to pass start:%dcm to pass entry:%dcm  path lengh:%dcm  after distance:%dcm \n",robot.GetPathDistances(1),robot.GetPathDistances(2),robot.GetPathDistances(3),robot.GetPathDistances(4));
     robot.RequestNarrowPathEchos;
     pause(3);
     retry=0;
     while (robot.pendingNarrowPathEchos && retry <5)
              robot.RequestNarrowPathEchos;
               retry++;
              pause(3);
    endwhile
    pathEchos=[];
    for i=1:8
          pathEchos=[pathEchos;[robot.GetPathEchosRight(i-1),robot.GetPathEchosLeft(i-1)]];
    endfor
     zoneNumber=1;
     farther=true;
     forward=true;
     [apRobot,robot,retCode] = UpdateLocationFromNarrowPath(apRobot,robot,zoneNumber,pathDistances,pathEchos,farther,forward);
     apGet(apRobot,"location")
 endif
endfunction