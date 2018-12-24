function [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,plotOn,scanId)
  %{
  send scan resquest to the robot and wait for completion
  %}
  action="resume";
  if (!exist("plotOn"))
    plotOn=false;
  endif
  if (!exist("scanId"))
    scanId=0;
  endif
  printf(mfilename);
  printf(" Scan 360 requested  *** ")
  printf(ctime(time()));									
  robot.Scan360Id(scanId);
  apRobot = setfield(apRobot,"waitFor",robot.scanEnd);     
  [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,0);
  if (retCode==-99)
        action="stop..";
        return;                 
   endif
   if (retCode==0)
     if(robot.scanReceiveCount!=apGet(apRobot,"nbPulse"))
      retCode=robot.scanReceiveCount;
      [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
      if (action=="retry.")
         printf(mfilename);
          printf(" Scan 360 retry ounce *** ")
          printf(ctime(time()));									
          robot.Scan360Id(scanId);
          apRobot = setfield(apRobot,"waitFor",robot.scanEnd);     
          [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,0);
          if (retCode==-99)
                action="stop..";
                return;                 
           endif
           if (retCode==0)
             if(robot.scanReceiveCount!=apGet(apRobot,"nbPulse"))
                retCode=-1;
                action="stop..";
                return;
              endif
           endif
       endif
      endif
     else
      action="stop..";
      %{
         [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
         if (action=="stop..")
               return;
         endif
         %}
    endif
  endfunction