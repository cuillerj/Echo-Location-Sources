function [apRobot,robot] = ApDetermineCurrentLocation(apRobot,robot,manualMode);
  printf("ApDetermineCurrentLocation\n")
  locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh");  % limit over wich location quality is reliable
  locationProbThresholdLow=apGet(apRobot,"locProbThresholdLow");  % limit under wich location quality is not reliable
  if (!exist("manualMode"))  % automatic location discovery is default mode 
     manualMode=true;
  endif
  realMode=apGet(apRobot,"realMode");
  if (manualMode==true )   % terminal input expected
      validate=false;
      while (validate==false)
        initialLocation(1)=input("enter current location X: ");
        initialLocation(2)=input("enter current location Y: ");
        initialLocation(3)=input("enter current location heading: ");
        initialProb=input("enter current location probability: ");
        printf("do you confirm this location: X=%d Y=%d H=%d Prob=%d ? ",initialLocation(1),initialLocation(2),initialLocation(3),initialProb);
        validate=yes_or_no("yes or no");
        [available,retCode] = ApQueryCartoAvailability(apRobot,initialLocation,0,1);
        if(!available)
          printf("current location not available: X=%d Y=%d H=%d ? ",initialLocation(1),initialLocation(2),initialLocation(3));
          validate=false;
        endif
      end
    apRobot = setfield(apRobot,"location",initialLocation);
    apRobot = setfield(apRobot,"gyroLocation",[initialLocation(1),initialLocation(2),initialLocation(3)*pi()/180]);
    apRobot = setfield(apRobot,"locationProb",initialProb);
    if (realMode)  % real mode running
         robotStatus=robot.runningStatus;
         if (robotStatus<=0)
          printf("wait for robot to be ready  *** ");
          printf(ctime(time()));
         endif
        while (robotStatus<=0)              % wait for robot to be ready
          robotStatus=robot.runningStatus;
          pause(2);
          printf(".");
        end
    endif
    [apRobot,robot] = ApUpdateHardLocation(apRobot,robot,initialLocation,initialProb);
    if (robot.currentLocProb>=locationProbThresholdLow) %  the location stored inside the robot must be taken into account
%        apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
 %       apRobot = setfield(apRobot,"locationProb",robot.currentLocProb);
        if (robot.currentLocProb>=locationProbThresholdHigh) % we can rely on the location stored inside the robot 
            printf(mfilename);
            printf(" hard robot location is good: %d *** ",robot.currentLocProb);
            printf(ctime(time()));
            return  
        endif
        printf(mfilename);        
        printf(" hard robot location is poor: %d *** ",robot.currentLocProb);
        printf(ctime(time()));
        return    
    else
        printf(mfilename);
        printf(" we need to use echo scan to determine the robot location  *** ");
        printf(ctime(time())); 
           % call location code
    endif
 else
    % automatic locationto be devlopped

    return
  endif

endfunction