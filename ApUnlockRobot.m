function [apRobot,robot,unlocked,retCode] = ApUnlockRobot(apRobot,robot,loopCount)
  %{
   This function look for a way to move straight forward and backward based on a front back ping
  %}
  flag=bitget (robot.IRMap, 8:-1:8);
  rotationType=2;  % select 1="northOrientation" 2="gyroscope" 3="wheels"
  printf(mfilename); 
  printf(" starting unlock process ***");
  printf(ctime(time()));
  minDistToBeDone=apGet(apRobot,"minDistToBeDone");
  rot=0;
  dist=0;
  if (flag==0)  % IR detection
      unlocked=false;
       retCode=0;
      [apRobot,robot,obstacleHeading,IRMap] = ApCheckIRRobot(apRobot,robot);
      if(sum(IRMap)==0) %  no more obstacle
            printf(mfilename); 
            printf(" no longer IR obstacle detected ***");
            printf(ctime(time()));
            unlocked=true;
       endif
      if(obstacleHeading ==0)
          dist=-2*minDistToBeDone;
          apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,0,dist]]);
          [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,dist,true,1);
           apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
           [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
           apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,3,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeMove]]);
           apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
          if (retCodeMove==0)
                unlocked=true;
            endif
      elseif(obstacleHeading ==180)
            dist=2*minDistToBeDone;
            apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,0,dist]]);
           [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,dist,true,1);
            apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
           [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
           apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,3,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeMove]]);
            apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
            if (retCodeMove==0)
               unlocked=true;
            endif
      else
          dist=-2*minDistToBeDone;
          apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,0,dist]]);
          [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,dist,true,1);
          apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
          [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
           apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,3,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeMove]]);
          apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
           if (retCodeMove!=0)
                  printf(mfilename); 
                  printf(" Unclock failed ***");
                  printf(ctime(time()));
                  retCode=-1;
                return
           endif
          [rotation,lenToDo,forward] = ApOptimizeMoveToDo(-obstacleHeading,0,0);  
          rot=rotation;
          apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,rotation,0]]);
          [apRobot,robot,retCodeMove,action]=ApRobotRotate(apRobot,robot,rotation,rotationType,1);
           apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
           [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
           apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,3,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeMove]]);
           apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
           if (retCodeMove==0)
                unlocked=true;
            endif
      endif
      if (unlocked==true)
   %      [apRobot,robot] = ApMoveParticles(apRobot,robot,rot,dist,1)
        printf(mfilename); 
        printf(" Unlock successfull ***");
        printf(ctime(time()));
        retCode=0;
        return
     endif
  else   % echo detection
      servoFrontHeading=90;
      forward=true;
      plotOn=true;
      nbMesurementByTrain=getNbStepsRotation();
      robot.RobotAlignServo(servoFrontHeading);
      apRobot = setfield(apRobot,"waitFor",robot.servoAlignEnd);     
      [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,0);
      apRobot = setfield(apRobot,"waitFor",robot.pingFBEnd);
      robot.PingEchoFrontBack();  
      [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,0);
      securityLenght=apGet(apRobot,"securityLenght");
      if(robot.GetPingAngle()!=servoFrontHeading)
        retCode=-1;
      endif
      distFront=robot.GetPingFront();
      distBack=robot.GetPingBack();
      marg=[distFront-(apGet(apRobot,"frontLenght")+securityLenght),distBack-(apGet(apRobot,"backLenght")+securityLenght)];
      [maxValue,idxMax]=max(marg);
      if (maxValue >= 3*securityLenght) 
        moveToDo=min(maxValue-securityLenght,3*securityLenght);
        if (idxMax==2)
          moveToDo=-moveToDo;
        endif
      elseif (distFront==0)
          moveToDo=securityLenght;
      elseif (distBack==0)
          moveToDo=-securityLenght;
      else
          printf(mfilename); 
          printf(" No possible move ***");
          printf(ctime(time()));
          retCode=-1;
          unlocked=false; 
          return;
      endif
      apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,0,moveToDo]]);
      [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,moveToDo,forward,plotOn);
      apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
      [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
      apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,3,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeMove]]);
      apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
      if (retCodeMove==0)
        printf(mfilename); 
        printf(" Unlock successfull ***");
        printf(ctime(time()));
        retCode=0;
        unlocked=true;  
      else
        printf(mfilename); 
        printf(" Move failed ***");
        printf(ctime(time()));
        retCode=-1;
        unlocked=false; 
      endif
  endif
endfunction