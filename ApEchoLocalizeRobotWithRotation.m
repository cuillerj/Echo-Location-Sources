function [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithRotation(apRobot,robot,nbPred,heading,plotOn)
% probability defined according to 28 tests
% first is right: =60% (17/28)
% second is right: =18% 5/28
% third is rignt: =4% 1/28
% fourth is right: = 8% 2/28
% fifth is right: =8% 2/28
% not found in first five =4% 1/28
%probRef=[60,18,06,06,06,04];
  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()))
  if (!plotOn)
      plotOn=false;
   endif
  nbPred=min(10,nbPred);
  
  probRef=[20,15,12,10,8,5,2,2,2,1,1];
  retCode=9;
  posProb=0;
  posX=0;
  posY=0;
  posAngle=0;
  [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,plotOn);

  if (retCode==0)
    printf(mfilename);
    printf("  scan ended  ***  ")
    printf(ctime(time()))
    robot.SetRunningStatus(0);
    [apRobot,robot,locX,locY,locAngle,locCost] = ApAnalyseLastScanRotation(apRobot,robot,nbPred,heading,plotOn);
%    [X,Y,Angle,Cost]=ApAnalyseLastScanRotation(robot,false,nbPred,heading,plotOn);
    posX=locX;
    posY=locY;
    posH=locAngle;
    for i=1:min(size(posX,2),size(probRef,2));
      posProb(i)=probRef(i);
    endfor
  endif
endfunction