function [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,plotOn)
  %{
  send scan resquest to the robot and wait for completion
  %}
  action="resume";
  if (!exist("plotOn"))
    plotOn=false;
  endif
  printf(mfilename);
  printf(" Scan 360 requested  *** ")
  printf(ctime(time()));									
  robot.Scan360();
  apRobot = setfield(apRobot,"waitFor",robot.scanEnd);     
  [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,0);
  if (retCode==-99)
        action="stop..";
        return;                 
   endif
   if (retCode==0)
         aligned=true;
     else
         [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
         if (action=="stop..")
               return;
         endif
    endif
  endfunction