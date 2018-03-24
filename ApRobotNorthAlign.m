function [apRobot,robot,retCode,action]=ApRobotNorthAlign(apRobot,robot,NorthHeading,alignParticles,plotOn)
  
  if (!exist("alignParticles"))
    alignParticles=false;
  endif
  printf(mfilename);
  printf(" align particles:%d *** ",alignParticles)
  printf(ctime(time()))
  currentShiftNorthOrientation=apGet(apRobot,"currentShiftNorthOrientation");
  currentNorthOrientationReference=apGet(apRobot,"currentNorthOrientationReference");
  aligned=false;
  debugOn=false;
  action="......";
  retCode=-1;
  if(alignParticles)
      lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
  endif
  saveNO=robot.northOrientation;
  printf(mfilename);
  printf(" NO align robot Current NO: %d  expected N Heading: %d  *** ",saveNO,NorthHeading+currentShiftNorthOrientation)
  printf(ctime(time()));									
  robot.NorthAlign(NorthHeading+currentShiftNorthOrientation);  
  apRobot = setfield(apRobot,"waitFor",robot.northAlignEnd);
  if (alignParticles)
    [apRobot,robot] = ApNorthAlignParticles(apRobot,robot,mod(currentNorthOrientationReference-NorthHeading,360),plotOn);
       %         [apRobot,robot]=ApMoveParticles(apRobot,robot,round(rotationToDo),0,plotOn);
  endif       
  [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,debugOn);
 % [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[northAlign,retCode],1);
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
  NO=robot.RefreshNorthOrientation();
  apRobot = setfield(apRobot,"waitFor",robot.robotNOUpdated);
  [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);       % wait for updated information from robot
  NO=robot.northOrientation;
  printf(mfilename);
  printf(" New NO: %d  *** ",robot.northOrientation);
  printf(ctime(time()));	
  endfunction