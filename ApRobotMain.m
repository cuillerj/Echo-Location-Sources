  function [apRobot,robot] = ApRobotMain(flatLogRegMode,currentInputMode,destInputMode,realMode,plotValue);
  %{
  start apRobot octave and java code
  parameters are: 
    flatLogRegMode to determine logistice regression mode flat (default) or rotated
    currentInputMode to determine the way to set the current staring location - 0: automatic localization requested, 1 manual location requested
    destInputMode to determine the way to set the destination - 0: hard coded 1: terminal input (default) other to be done
    realMode to determine the running mode 1: means real (default)  mode 0: means simulation mode 
    plotValue 0 means no plot, 1 high means level graph to be plotted, 2 means all graph to be plotted
  %}
    cd c:\Users\jean\Documents\Donnees\octave\robot;
    more off
    nbPred=5;                    % define the number of predictions that will be compute for each 360° scan
    plotOn=true;                 % if true graphics will be provided (reduces performance)
    plotOff=false;               % non graphic
    printf(ctime(time()))
    particlesNumber=1000;        % define number of particles use with particules filter
   % noise parameters for simulator
    noiseLevel=0.0;                 % coefficent (float) of noise of simualtion mode 0 no noise
    noiseRetCode=1;                 % boolean (0 move 100% completed 1 move can be incompleted)
    noiseRetValue=12;               % range of noised retcode in wich a random retcode is choosen
    startingState=true;
    validate=true;
    loopCount=0;
  % below to trace events
    trajectory=[];
    traceDet=[];
    traceMove=[];
    traceNext=[];
    traceRobot=[];
    traceEcho=[];
    degreUnit=false;
    radianUnit=true;
    debugOn=true;
    debugOff=false;
   %
    if (!exist("flatLogRegMode"))  % flat logistic regression is default mode 
       flatLogRegMode=true;
    endif
    if (!exist("destInputMode"))  % terminal input is default mode 
       destInputMode=1;
    endif
    if (!exist("currentInputMode"))  % terminal input is default mode 
       currentInputMode=1;
    endif
    if (!exist("realMode"))  % real is default mode 
       realMode=1;
    endif
    if (!exist("plotValue"))  % real is default mode 
       plotValue=1;
    endif
    simulationMode=!realMode;
    printf(mfilename);
    printf(" starting mode flatLogRegMode: %d currentInputMode:%d destInputMode:%d realMode:%d plotValue:%d \n",flatLogRegMode,currentInputMode,destInputMode,realMode,plotValue);
  % init context octave and java objects
    [apRobot,robot] =ApInitApRobot(flatLogRegMode);
    %locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh") 
    carto=apGet(apRobot,"carto");
    img=apGet(apRobot,"img");
    rotationType=apGet(apRobot,"rotationType");
    apRobot = setfield(apRobot,"simulationMode",simulationMode);
    apRobot = setfield(apRobot,"realMode",realMode);
    shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter");
    if (realMode)
      robot.LaunchBatch();        % call java method to start all batchs
    else
      robot.LaunchSimu();        % call java method to start only simulator
    endif
    if (simulationMode)
      robot.noiseLevel=noiseLevel;                 % coefficent (float) of noise of simualtion mode 0 no noise
      robot.noiseRetCode=noiseRetCode;                 % boolean (0 move 100% completed 1 move can be incompleted)
      robot.noiseRetValue=noiseRetValue;               % range of noised retcode in wich a random retcode is choosen
      printf(mfilename);
      printf(" Simulation noiseLevel:%f noiseRetCode:%d noiseRetValue:%d *** ",robot.noiseLevel,robot.noiseRetCode,robot.noiseRetValue);
      printf(ctime(time()));
    endif
    % set destination
    noMoreDestination=false;
    %{
      loop until no more destination 
    %}
    while (!noMoreDestination)          
      if (!startingState)
        noMoreDestination = ApAskForNextDestination();
      endif
      if (noMoreDestination)
        printf("end due to no more destination  *** ");
        printf(ctime(time())); 
        break;
      endif
      apRobot=ApSetDestinationLocation(apRobot,destInputMode);  % set the destination
  %    apRobot = setfield(apRobot,"destination",destLocation);
      if (startingState)  % use currentInputMode for setting the first current location
        manualMode=currentInputMode;
        else
        manualMode=false;
      endif
      reliableLocation=false;
      while (!reliableLocation)  % loop until current location quality is good enough
        %locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh") 
        [apRobot,robot]=ApDetermineCurrentLocation(apRobot,robot,manualMode); 
        if (apGet(apRobot,"locationProb")>=apGet(apRobot,"locProbThresholdHigh"))
              break;
        endif
      end
      currentLocation=apGet(apRobot,"location");
      currentLocationProb=apGet(apRobot,"locationProb");
      trajectory=[trajectory;currentLocation];
      printf(mfilename);
      printf(" robot location is X:%d Y:%d orientation: %d. prob:%d *** ",currentLocation(1),currentLocation(2),currentLocation(3),currentLocationProb);
      printf(ctime(time()));
      if (plotValue>=3)
        plotReq=true;
      else
        plotReq=false;
      endif
      if (!exist("particles"))  % flat logistic regression is default mode 
        [apRobot]= ApCreateLocatedParticles(apRobot,particlesNumber,plotReq);  % create particles for particles filter
        startingState=false;
        printf(mfilename);   
        printf(" objects and particles filter ready  *** ");
        printf(ctime(time())); 
      endif
      if (plotValue>=1)
         plotOn=true;
       else
        plotOn=false;
      endif
 %   [apRobot,robot,next,direct,forward] = ApComputeNextStepToTarget(apRobot,robot,plotOn);
    while(!ApTargetReached(apRobot,robot))
        if (apGet(apRobot,"newTarget"))
          [apRobot,robot,next,forward] = ApComputeOptimalPath(apRobot,robot,apGet(apRobot,"destLocation"),plotOn)
        else
          [apRobot,robot,next,closestIdx] = ApFindClosestPathLocation(apRobot,robot,apGet(apRobot,"location")) 
        endif
        if (forward==0)
          % temporarly end program - need to call automatic localisation
           printf(mfilename);
           printf(" end no path found. *** ")
           printf(ctime(time()))
           AStarShowStep(trajectory,"determined trajectory");
           save ("-mat4-binary","traceDet.mat","traceDet");
           save ("-mat4-binary","traceMove.mat","traceMove");
           save ("-mat4-binary","traceNext.mat","traceNext");
           save ("-mat4-binary","traceRobot.mat","traceRobot");
           save ("-mat4-binary","traceEcho.mat","traceEcho");
           return
         endif
         loopCount=loopCount+1;
         printf(mfilename);
         printf(" Step nb:%d X:%d Y:%d . *** ",loopCount,next(1),next(2))
         printf(ctime(time()))
         [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward)
         apRobot = setfield(apRobot,"saveLocation",apGet(apRobot,"location"));
         targetHeading=apGet(apRobot,"location")(3)+rotationToDo
         traceNext=[traceNext;[time,loopCount,next(1),next(2)]];     
         traceMove=[traceMove;[time,loopCount,rotationToDo,lenToDo]];
         printf("rotation:%d distance:%d *** ",rotationToDo,lenToDo);
         printf(ctime(time()));
         if (plotValue>=2)
             plotOn=true;
           else
            plotOn=false;
         endif
         if (rotationToDo!=0)
          robot.Horn(2);  % horn x seconds
          pause(3);
          [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOn);
          if (action=="stop..")
            printf(mfilename);
             printf(" stop due to pb rotation")
             printf(" *** ");
             issue==true;
             lenToDo=0;
          endif
         endif
         gyroLenToDo=lenToDo;
         robot.Horn(2);  % horn 2 seconds
         pause(3);
         retCode=0;
         if (lenToDo!=0)
            [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOn);
            traceRobot=[traceRobot;[time,loopCount,2,robot.GetHardPosX(),robot.GetHardPosY(),+robot.GetHardHeading(),robot.GetGyroHeading()],retCode];
         endif
         while (retCode==robot.moveKoDueToNotEnoughSpace) % loop decrease move length until move possible
             if (robot.retCodeDetail>=minDistToBeDone)
                {
                 lenToDo=robot.retCodeDetail;
                 printf(mfilename);
                 printf(" try new length  x:%d *** ",lenToDo)
                 printf(ctime(time()))
                [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,plotOn);
                 traceRobot=[traceRobot;[time,loopCount,2,robot.GetHardPosX(),robot.GetHardPosY(),+robot.GetHardHeading(),robot.GetGyroHeading()],retCode];
                 gyroLenToDo=lenToDo; 
                 nextLoc(1)=round(saveLocation(1)+cos((saveLocation(3)+rotationToDo)*pi()/180)*lenToDo)
                 nextLoc(2)=round(saveLocation(2)+sin((saveLocation(3)+rotationToDo)*pi()/180)*lenToDo)
                 nextLoc(3)=mod(saveLocation(3)+rotationToDo,360)
                 apRobot = setfield(apRobot,"nextLocation",nextLoc);
                 }           
              else{
                 issue=true;
                 printf(mfilename);
                 printf(" no possible move max distance:%d *** ",robot.retCodeDetail);
                 printf(ctime(time()));
                 retCode=99;
                  }
              endif       
         end
          apRobot = setfield(apRobot,"subsytemLeft",[robot.BNOLeftPosX,robot.BNOLeftPosX,robot.BNOLocHeading]);
          apRobot = setfield(apRobot,"subsytemRight",[robot.BNORightPosX,robot.BNORightPosX,robot.BNOLocHeading]);
          printf(mfilename);    
          printf("Subsystem location leftX:%d leftY:%d LHeading:%d rightX:%d rightY:%d RHeading:%d",apGet(apRobot,"subsytemLeft")(1),apGet(apRobot,"subsytemLeft")(2),apGet(apRobot,"subsytemLeft")(3),apGet(apRobot,"subsytemRight")(1),apGet(apRobot,"subsytemRight")(2),apGet(apRobot,"subsytemRight")(3));
          printf(" *** ");
          printf(ctime(time()));
          % get and store BNO subsystem data
          gyroBasedH=mod(apGet(apRobot,"gyroLocation")(3)+robot.GetGyroHeading()*pi()/180,2*pi());
          gyroBasedX=round(apGet(apRobot,"gyroLocation")(1)+gyroLenToDo*cos(gyroBasedH)+shiftEchoVsRotationCenter*cos(gyroBasedH));
          gyroBasedY=round(apGet(apRobot,"gyroLocation")(2)+gyroLenToDo*sin(gyroBasedH)+shiftEchoVsRotationCenter*sin(gyroBasedH));
          gyroBasedX=gyroBasedX-shiftEchoVsRotationCenter*cos(gyroBasedH); % set position to rotation center
          gyroBasedY=gyroBasedY-shiftEchoVsRotationCenter*sin(gyroBasedH); % set position to rotation center
          gyroBasedLoc=[gyroBasedX,gyroBasedY,gyroBasedH]
          apRobot = setfield(apRobot,"gyroLocation",gyroBasedLoc);
          apRobot = setfield(apRobot,"hardLocation",[robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHeading()]);
          % prepare data for localization determination

          [available,retCode]=ApQueryCartoAvailability(apRobot,[robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading()],degreUnit,debugOn);
          if(available==false)
           probHardMoveOk=1;  % the hard position is theoriticaly not possible
           printf("the hard position is theoriticaly not possible. *** x:%d y:%d h:%d ",robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading())
           printf(ctime(time()))
           manualMode=true; % temporarly manual mode
           [apRobot,robot] = ApDetermineCurrentLocation(apRobot,robot,manualMode)
          endif
           [apRobot,robot,northOrientation,headingNO] = ApSpaceNorthOrientation(apRobot,robot); % get heading based on compass
           if (!simulationMode)
               newX=[nextX,robot.GetHardPosX(),gyroBasedX,subsystemLeftX,subsystemRightX];
               newY=[nextY,robot.GetHardPosY(),gyroBasedY,subsystemLeftY,subsystemRightY];
          else
               newX=[nextX,robot.GetHardPosX(),gyroBasedX];
               newY=[nextY,robot.GetHardPosY(),gyroBasedY];
          endif
          gyroBasedX=gyroBasedX-shiftEchoVsRotationCenter*cos(gyroBasedH); % set position to rotation center
          gyroBasedY=gyroBasedY-shiftEchoVsRotationCenter*sin(gyroBasedH); % set position to rotation center
          if (rotationType==2)
              newH=[mod(360+gyroBasedH*180/pi(),360)];
           else
              newH=[mod(360+robot.GetHardHeading(),360)];
           endif
           printf(mfilename); 
           printf (" Heading hard H:%d  NO:%d Theoretical H:%d  SpaceNO:%d RobotNO:%d Gyro:%d*** ",robot.GetHardHeading(),northOrientation,targetHeading,headingNO,robot.northOrientation,robot.GetGyroHeading());
           printf(ctime(time()))
           [apRobot,robot,weight,retValue,echo] = ApTestLocationEchoConsistancy(apRobot,robot,newX,newY,newH);
           if (retValue==0)
               traceEcho=[traceEcho;[time,loopCount,newX,newY,weightEcho,echo,quality]];
           else
               printf(mfilename); 
               printf(" no echo consistancy data *** ");
               printf(ctime(time()));
               for i=1:col	
                   weightEcho(i)=1;
               endfor;	                
            endif
          
       end
    end
  endfunction