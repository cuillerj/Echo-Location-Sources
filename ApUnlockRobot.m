function [apRobot,robot,unlocked,retCode] = ApUnlockRobot(apRobot,robot)
  %{
   This function look for a way to move straight forward and backward based on a front back ping
   
  %}
  printf(mfilename); 
  printf(" starting unlock process ***");
  printf(ctime(time()));
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
  [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,moveToDo,forward,plotOn);
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
endfunction