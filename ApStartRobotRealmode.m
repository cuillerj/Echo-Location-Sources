 function [apRobot,robot] = ApStartRobotRealmode(flatLogRegMode,plotValue)
  printf(mfilename);
  printf("   ***   ");
  printf(ctime(time()))
 more off;
 realMode=true;
 simulationMode=!realMode;
 cpu1=cputime()
 if (!exist("flatLogRegMode"))  % flat logistic regression is default mode 
      flatLogRegMode=true;
  endif

  if (!exist("plotValue"))  % real is default mode 
      plotValue=1;
  endif
  [apRobot,robot] =ApInitApRobot(flatLogRegMode,realMode);
  apRobot = setfield(apRobot,"simulationMode",simulationMode);
  apRobot = setfield(apRobot,"realMode",realMode);
  carto=apGet(apRobot,"carto");
  img=apGet(apRobot,"img");
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  rotationType=apGet(apRobot,"rotationType");  %  robot rotation 1: based on NO, 2: based on gyroscope, 3: based on encoder
  shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter")/10;
  currentNorthOrientationReference=apGet(apRobot,"currentNorthOrientationReference");
  robot.LaunchBatch();        % call java method to start all batchs
  robotStatus=robot.runningStatus;
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
   pause(2);
  [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);  % download parameters inside the robot
endfunction