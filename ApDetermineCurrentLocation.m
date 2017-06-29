function [apRobot,robot,stopRequested] = ApDetermineCurrentLocation(apRobot,robot,manualMode);
  locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh");  % limit over wich location quality is reliable
  locationProbThresholdLow=apGet(apRobot,"locProbThresholdLow");  % limit under wich location quality is not reliable
  stopRequested=false;
  if (!exist("manualMode"))  % automatic location discovery is default mode 
     manualMode=true;
  endif
  realMode=apGet(apRobot,"realMode");
  if (manualMode==true )   % terminal input expected
      validate=false;
      while (validate==false)
        initialLocation(1)=input("enter current location X (0 to stop): ");
        initialLocation(2)=input("enter current location Y (0 to stop): ");
        initialLocation(3)=input("enter current location heading (0 to stop): ");
        initialProb=input("enter current location probability (0 to stop): ");
        printf("do you confirm this location: X=%d Y=%d H=%d Prob=%d ? ",initialLocation(1),initialLocation(2),initialLocation(3),initialProb);
        validate=yes_or_no("yes or no");
        if (initialLocation(1)==0 && initialLocation(2)==0 &&initialLocation(3)==0 && initialProb==0)
          printf(mfilename);
          printf(" stop requested  *** ");
          printf(ctime(time()));
          stopRequested=true;
          return
        endif
        [available,retCode] = ApQueryCartoAvailability(apRobot,initialLocation,0,1);
        if(!available )
          printf(mfilename);
          printf(" current location not available: X=%d Y=%d H=%d ? ",initialLocation(1),initialLocation(2),initialLocation(3));
          printf(ctime(time()));
          validate=false;
        endif
      end
    apRobot = setfield(apRobot,"location",initialLocation);
    apRobot = setfield(apRobot,"gyroLocation",initialLocation);
    apRobot = setfield(apRobot,"locationProb",initialProb);
    if (realMode)  % real mode running
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