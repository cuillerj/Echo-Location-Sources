function [apRobot,robot,retCode,action]=ApRobotNorthAlign(apRobot,robot,NorthHeading,alignParticles,plotOn)
  
  if (!exist("alignParticles"))
    alignParticles=false;
  endif
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
  if (retCode==-1) % timeout
       robot.NorthAlign(NorthHeading+currentShiftNorthOrientation);  % retry once
       [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,debugOn);
  endif
  if (retCode==0)
         aligned=true;
     elseif (retCode==robot.moveKoDueToNotEnoughSpace)
          apRobot = setfield(apRobot,"particles",lastParticles);
          %[apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,plotOn);
     else
         [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
         if (action=="stop..")
               return;
         endif
  endif

 % apRobot = setfield(apRobot,"waitFor",robot.robotNOUpdated);
  %[apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);       % wait for updated information from robot
  prevNO=robot.RefreshNorthOrientation();
  NO=999;
  count=0;
  while((NO==-1 || NO==999) && count <10)
    NO=robot.RefreshNorthOrientation();
    printf(".%d",NO);
    pause(2);
    count++;
  endwhile
  printf("\n");
  printf(mfilename);
  if (NO!=999)
      printf(" New North orientation: %d  *** ",NO);
  else
       printf(" North orientation: timeout *** ");
       retCode=-1;
  endif
  printf(ctime(time()));	
  # for debug
  #{
  validate=false;
  while (validate==false)
    printf(mfilename);
    printf("   Is robot north aligned ? else do it manually ***");
    validate=yes_or_no("yes or no");
    apRobot = setfield(apRobot,"waitFor",robot.robotNOUpdated);
    [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);       % wait for updated information from robot
    while(NO==-1)
      NO=robot.RefreshNorthOrientation();
  #    apRobot = setfield(apRobot,"waitFor",robot.robotNOUpdated);
      [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);       % wait for updated information from robot
      pause(1)
    endwhile
    #}

   #end
  endfunction