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
    noiseLevel=0.05;                 % coefficent (float) of noise of simualtion mode 0 no noise
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
    delete('traceDet.mat');
    delete('traceMove.mat');
    delete('traceNext.mat');
    delete('traceRobot.mat');
    delete('traceEcho.mat');
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
    if (!exist("realMode"))  % simulation is default mode 
       realMode=0;
    endif
    if (!exist("plotValue"))  % real is default mode 
       plotValue=1;
    endif
    simulationMode=!realMode;
    printf(mfilename);
    printf(" starting mode flatLogRegMode: %d currentInputMode:%d destInputMode:%d realMode:%d plotValue:%d \n",flatLogRegMode,currentInputMode,destInputMode,realMode,plotValue);
  % init context octave and java objects
    if (!exist("robot"))  % real is default mode 
       robot=true;
    endif
    [apRobot,robot] =ApInitApRobot(flatLogRegMode);
    %locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh") 
    carto=apGet(apRobot,"carto");
    img=apGet(apRobot,"img");
    rotationType=apGet(apRobot,"rotationType");
    apRobot = setfield(apRobot,"simulationMode",simulationMode);
    apRobot = setfield(apRobot,"realMode",realMode);
    shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter")/10;
    if (realMode)
      robot.LaunchBatch();        % call java method to start all batchs
    else
      robot.LaunchSimu();        % call java method to start only simulator
    endif
    if (simulationMode)
      robot.noiseLevel=noiseLevel;                 % coefficent (float) of noise of simualtion mode 0 no noise
      robot.noiseRetCode=noiseRetCode;                 % boolean (0 move 100% completed 1 move can be incompleted)
      robot.noiseRetValue=noiseRetValue;               % range of noised retcode in wich a random retcode is choosen
      echoBalance=[1,0.95,1.05];  % theoritical , encoder , gyroscope
      printf(mfilename);
      printf(" Simulation noiseLevel:%f noiseRetCode:%d noiseRetValue:%d *** ",robot.noiseLevel,robot.noiseRetCode,robot.noiseRetValue);
      printf(ctime(time()));
    else
       echoBalance=[1,0.95,1.05,1.1,1.1];  % theoritical , encoder , gyroscope theo , BNO Left, BNO Right      
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
        saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho);
        return;
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
        [apRobot,robot,stopRequested]=ApDetermineCurrentLocation(apRobot,robot,manualMode);
        if (stopRequested)
          saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho);
          return;
        endif
        if (apGet(apRobot,"locationProb")>=apGet(apRobot,"locProbThresholdHigh"))
              break;
        endif

      end
      if (realMode && startingState)
         [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);  % download paramters inside the robot
      endif  
      currentLocation=apGet(apRobot,"location");
      currentLocationProb=apGet(apRobot,"locationProb");
      trajectory=[apGet(apRobot,"trajectory");currentLocation];
      apRobot = setfield(apRobot,"trajectory",trajectory);
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
          [apRobot,robot,next,forward] = ApComputeOptimalPath(apRobot,robot,apGet(apRobot,"destination"),plotOn);
        endif
        [apRobot,robot,next,closestIdx] = ApFindClosestPathLocation(apRobot,robot,apGet(apRobot,"location")); 
        apRobot = setfield(apRobot,"nextLocation",next);
        if (forward==0)
          % temporarly end program - need to call automatic localisation
           printf(mfilename);
           printf(" end no path found. *** ")
           printf(ctime(time()))
           AStarShowStep(trajectory,"determined trajectory");
           saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho);
           return;
        endif
         loopCount=loopCount+1;
         printf(mfilename);
         printf(" Step nb:%d X:%d Y:%d . *** ",loopCount,next(1),next(2))
         printf(ctime(time()))
         [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward);
         apRobot = setfield(apRobot,"saveLocation",apGet(apRobot,"location"));
         targetHeading=mod(apGet(apRobot,"location")(3)+rotationToDo,360);
         traceNext=[traceNext;[time,loopCount,next(1),next(2)]];     
         traceMove=[traceMove;[time,loopCount,rotationToDo,lenToDo]];
         printf(mfilename);
         printf(" rotation:%d *** ",rotationToDo);
         printf(ctime(time()));
         if (plotValue>=2)
             plotOn=true;
           else
            plotOn=false;
         endif
         if (rotationToDo!=0)     
          robot.Horn(2);  % horn x seconds
          pause(3);
          [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOff);
           if (action=="retry.")
             printf(mfilename);
             printf(" no rotation >> retry *** ");
             printf(ctime(time()));
             issue=true;
             lenToDo=0;
             apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"location"));
          endif
          if (action=="stop..")
             printf(mfilename);
             printf(" stop due to pb rotation *** ");
             printf(ctime(time()))
             issue=true;
             lenToDo=0;
             apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"location"));
          endif
         endif
         gyroLenToDo=lenToDo;
         robot.Horn(2);  % horn 2 seconds
         pause(3);
         retCode=0;       
         if (lenToDo!=0)
            [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOff);
            traceRobot=[traceRobot;[time,loopCount,2,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading(),robot.GetGyroHeading(),retCode]];
         endif
         while (retCode==robot.moveKoDueToNotEnoughSpace && lenToDo!=0) % loop decrease move length until move possible
             if (robot.GetRetcodeDetail()>=apGet(apRobot,"minDistToBeDone"))
                {
                 lenToDo=floor(robot.GetRetcodeDetail()-apGet(apRobot,"securityLenght"))*sign(lenToDo);
                 printf(mfilename);
                 printf(" try new length  x:%d *** ",lenToDo)
                 printf(ctime(time()))
                [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,plotOn);
                traceRobot=[traceRobot;[time,loopCount,2,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading(),robot.GetGyroHeading(),retCode]];
                 gyroLenToDo=lenToDo; 
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
         if (retCode==-1)    % timeout
           printf(mfilename);
           printf(" move straight timeout");
           printf(ctime(time()));

           if (apGet(apRobot,"saveLocation")(1)==robot.GetHardPosX() && apGet(apRobot,"saveLocation")(2)==robot.GetHardPosY()) % no move
                  apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"saveLocation"));
                  apRobot = setfield(apRobot,"gyroLocation",apGet(apRobot,"saveLocation"));   
                  printf(mfilename);
                  printf(" hard position not changed >> no move *** ");
                  printf(ctime(time()));
             endif
         endif  
           [apRobot,robot]=ApUpdateLocations(apRobot,robot);  
           [apRobot,robot,northOrientation,headingNO] = ApSpaceNorthOrientation(apRobot,robot); % get heading based on compass
         if (realMode)
            printf(mfilename);    
            printf(" Subsystem location leftX:%d leftY:%d LHeading:%d rightX:%d rightY:%d RHeading:%d",apGet(apRobot,"subsytemLeft")(1),apGet(apRobot,"subsytemLeft")(2),apGet(apRobot,"subsytemLeft")(3),apGet(apRobot,"subsytemRight")(1),apGet(apRobot,"subsytemRight")(2),apGet(apRobot,"subsytemRight")(3));
            printf(" *** ");
            printf(ctime(time()));
          endif

          if (realMode)
               newX=round([apGet(apRobot,"nextLocation")(1),robot.GetHardPosX(),apGet(apRobot,"gyroLocation")(1),apGet(apRobot,"subsytemLeft")(1),apGet(apRobot,"subsytemRight")(1)]);
               newY=round([apGet(apRobot,"nextLocation")(2),robot.GetHardPosY(),apGet(apRobot,"gyroLocation")(2),apGet(apRobot,"subsytemLeft")(2),apGet(apRobot,"subsytemRight")(2)]);
          else
               newX=round([apGet(apRobot,"nextLocation")(1),robot.GetHardPosX(),apGet(apRobot,"gyroLocation")(1)]);
               newY=round([apGet(apRobot,"nextLocation")(2),robot.GetHardPosY(),apGet(apRobot,"gyroLocation")(2)]);
          endif
          gyroBasedX=apGet(apRobot,"gyroLocation")(1)-shiftEchoVsRotationCenter*cos(apGet(apRobot,"gyroLocation")(3)*pi()/180); % set position to rotation center
          gyroBasedY=apGet(apRobot,"gyroLocation")(2)-shiftEchoVsRotationCenter*sin(apGet(apRobot,"gyroLocation")(3)*pi()/180); % set position to rotation center
          if (rotationType==2)
              newH=[apGet(apRobot,"gyroLocation")(3)];
           else
              newH=[mod(360+robot.GetHardHeading(),360)];
           endif
           printf(mfilename); 
           printf (" Heading hard H:%d  NO:%d Theoretical H:%d  SpaceNO:%d RobotNO:%d Gyro:%d*** ",robot.GetHardHeading(),northOrientation,targetHeading,headingNO,robot.northOrientation,robot.GetGyroHeading());
           printf(ctime(time()))
           [apRobot,robot,weightEcho,retValue,echo,quality] = ApTestLocationEchoConsistancyVsDB(apRobot,robot,newX,newY,newH);
           if (retValue==0)
               traceEcho=[traceEcho;[time,loopCount,newX,newY,weightEcho,echo,quality]];
           else
               printf(mfilename); 
               printf(" no echo consistancy data *** ");
               printf(ctime(time()));
               for i=1:size(newX,2)	
                   weightEcho(i)=1;
               endfor;	                
            endif
            [nbRow,nbCol]=size(weightEcho);
            printf(mfilename);    
            printf(" best quality (0: one is perfect):%f ***",quality);
            printf(ctime(time()));
            weightEcho=weightEcho.*echoBalance;   %
            printf(mfilename);
            printf(" positions list: ");
            for i=1:size(newX,2)
              printf(" (%d,%d) ",newX(i),newY(i));
            end  
            for i=1:size(newX,2)
             [available,retCode]=ApQueryCartoAvailability(apRobot,[newX(i),newY(i),newH],degreUnit,debugOn);
             if (!available)
                printf(mfilename);
                printf(" positions problem: (%d,%d) ",newX(i),newY(i));
                printf(ctime(time()));
               weightEcho(i)=weightEcho(i)/2;
             endif
            end  
            printf(ctime(time())); 
            [apRobot,robot,detX,detY,detH] = ApDetermineRobotLocationWithParticlesGaussian(apRobot,robot,newX,newY,weightEcho,plotOn);
            traceDet=[traceDet;[time,loopCount,detX,detY,detH]];
            printf(mfilename); 
            printf(" determined X:%f Y:%f Heading:%f *** ",detX,detY,detH);
            printf(ctime(time()));
           [available,retCode]=ApQueryCartoAvailability(apRobot,[detX,detY,detH],degreUnit,debugOn);
            if(available==false)
               probHardMoveOk=1;  % the hard position is theoriticaly not possible
               printf(mfilename);    
               printf(" the determined position is theoriticaly not possible. *** x:%d y:%d h:%d ",detX,detY,detH)
               printf(ctime(time()))
               manualMode=true; % temporarly manual mode
               [apRobot,robot,stopRequested] = ApDetermineCurrentLocation(apRobot,robot,manualMode);
               if (stopRequested)
                  saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho);
                  return;
               endif
          endif           
            trajectory=[apGet(apRobot,"trajectory");[detX,detY,detH]];          
            apRobot = setfield(apRobot,"trajectory",trajectory);
            trajectory=[trajectory;[detX,detY,detH]];
            prob=round(robot.currentLocProb*.95);
            [apRobot,robot] = ApUpdateHardLocation(apRobot,robot,[detX,detY,detH],prob);
       end
    end
    function [] = saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho)
           cd c:\Users\jean\Documents\Donnees\octave\robot;
           printf(mfilename);    
           printf(" end program *** ");
           printf(ctime(time()))
           save ("-mat4-binary","traceDet.mat","traceDet");
           save ("-mat4-binary","traceMove.mat","traceMove");
           save ("-mat4-binary","traceNext.mat","traceNext");
           save ("-mat4-binary","traceRobot.mat","traceRobot");
           save ("-mat4-binary","traceEcho.mat","traceEcho");
           ApShowStep(apRobot,apGet(apRobot,"trajectory"),"Actual Trajectory");
           return;
    endfunction
  endfunction