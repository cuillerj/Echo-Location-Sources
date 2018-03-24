  function [apRobot,robot] = ApRobotGoToTarget(flatLogRegMode,currentInputMode,destInputMode,realMode,plotValue);
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
    more off;
    nbPred=5;                    % define the number of predictions that will be compute for each 360° scan
    plotOn=true;                 % if true graphics will be provided (reduces performance)
    plotOff=false;               % non graphic
    printf(ctime(time()))
    particlesNumber=300;        % define number of particles use with particules filter
    limitForTestLocationEchoConsistancy=15*15  % square of distance cm between the farest estimated points over that try to choose with the sonar
   % noise parameters for simulator
    noiseLevel=0.0;                 % coefficent (float) of noise of simualtion mode 0 no noise
    noiseRetCode=1;                 % boolean (0 move 100% completed 1 move can be incompleted)
    noiseRetValue=10;               % range of noised retcode in wich a random retcode is choosen
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
    issue=false;
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
    [apRobot,robot] =ApInitApRobot(flatLogRegMode,realMode);
    apRobot = setfield(apRobot,"simulationMode",simulationMode);
    apRobot = setfield(apRobot,"realMode",realMode);
    %locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh") 
    carto=apGet(apRobot,"carto");
    img=apGet(apRobot,"img");
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
    figure(1);    % figure 1 fixed for optimal path & determined locatio
		title ("optimal path (x) & determined location(+)");
		hold on;
		imshow(img,[]);       % insert map background
		[a,b]=size(img);
    c=max(a,b);
	  axis([1,b,1,a],"on","xy");
    %axis([1,c,1,c],"square","on","xy")
    hold on;
    if (plotValue>=2) % have to plot particles ?
      figure(2);      % figure 2 fixed for particles
		  title ("particles");
		  hold on;
		  imshow(img,[]);  % insert map background
		  [a,b]=size(img);
		  axis([1,b,1,a],"on","xy");
      hold on;
    endif
    if (plotValue>=3) % have to plot particles ?
      figure(3);      % figure 2 fixed for particles
		  title ("re-sample particles");
		  hold on;
		  imshow(img,[]);  % insert map background
		  [a,b]=size(img);
		  axis([1,b,1,a],"on","xy");
      hold on;
    endif
    rotationType=apGet(apRobot,"rotationType");  %  robot rotation 1: based on NO, 2: based on gyroscope, 3: based on encoder
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
      echoBalance=[1,0.95,1.05];  % theoritical , encoder , gyroscope theo
      printf(mfilename);
      printf(" Simulation noiseLevel:%f noiseRetCode:%d noiseRetValue:%d *** ",robot.noiseLevel,robot.noiseRetCode,robot.noiseRetValue);
      printf(ctime(time()));
    else
       echoBalance=[1,1,1,1.15,1.15];  % theoritical , encoder , gyroscope theo , BNO Left, BNO Right      
    endif
    pointLib=["theori:";"encoder:";"gyroTheo:";"BNOleft:";"BNORight:"];
    % set destination
    noMoreDestination=false;
    %{
      loop until no more destination 
    %}
    while (!noMoreDestination && robot.runningStatus >=0)          
      if (!startingState)
          for i=1:5
              robot.Horn(1);  % horn x seconds
              pause(2);
           end
        noMoreDestination = ApAskForNextDestination();
      endif
      if (noMoreDestination)
        printf("end due to no more destination  *** ");
        printf(ctime(time()));
        saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
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
          saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
          return;
        endif
        if (apGet(apRobot,"locationProb")>=apGet(apRobot,"locProbThresholdHigh"))
              break;
        endif

      end
      traceDet=[traceDet;[time,loopCount,apGet(apRobot,"location")(1),apGet(apRobot,"location")(2),apGet(apRobot,"location")(3)]];
      if (realMode && startingState)
         [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);  % download parameters inside the robot
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
    while(!ApTargetReached(apRobot,robot) && robot.runningStatus>=0)
        if (apGet(apRobot,"newTarget"))
          [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,apGet(apRobot,"destination"),plotOn);
        endif
        [apRobot,robot,next,closestIdx,forward] = ApFindClosestPathLocation(apRobot,robot,apGet(apRobot,"location"));
        if (closestIdx>0)   % found close enough point
          apRobot = setfield(apRobot,"nextLocation",next);
         else
           [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,apGet(apRobot,"destination"),plotOn);
           [apRobot,robot,next,closestIdx,forward] = ApFindClosestPathLocation(apRobot,robot,apGet(apRobot,"location"));
        endif
        if (forward==-9)
          % temporarly end program - need to call automatic localisation
           printf(mfilename);
           printf(" end no path found. *** ")
           printf(ctime(time()))
           AStarShowStep(trajectory,"determined trajectory");
           saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
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
         printf(mfilename);
         printf(" rotation:%d *** ",rotationToDo);
         printf(ctime(time()));
         traceMove=[traceMove;[time,loopCount,rotationToDo,lenToDo,forward]];
         if (plotValue>=2)
             plotOn=true;
           else
            plotOn=false;
         endif
         if (rotationToDo!=0)     
              robot.Horn(2);  % horn x seconds
              pause(5);
              if (robot.actionRetcode!=0)
                robot.ResetRobotStatus();
                pause(1);
              endif
              [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOff);
               if (action=="retry.")
                 printf(mfilename);
                 printf(" no rotation >> retry *** ");
                 printf(ctime(time()));
                 issue=true;
                 lenToDo=0;
                 apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"saveLocation"));
                 apRobot = setfield(apRobot,"gyroLocation",apGet(apRobot,"saveLocation"));
                 apRobot = setfield(apRobot,"lastRotation",0);
                 apRobot = setfield(apRobot,"lastMove",0);
                 apRobot = setfield(apRobot,"particles",apGet(apRobot,"lastParticles"));  % restaure particles
              endif
              traceRobot=[traceRobot;[time,loopCount,1,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading(),robot.GetGyroHeading(),apGet(apRobot,"subsytemLeft")(1),apGet(apRobot,"subsytemLeft")(2),apGet(apRobot,"subsytemLeft")(3),apGet(apRobot,"subsytemRight")(1),apGet(apRobot,"subsytemRight")(2),apGet(apRobot,"subsytemRight")(3),robot.actionRetcode]];
              if (action=="stop..")
                 printf(mfilename);
                 printf(" stop due to pb rotation *** ");
                 printf(ctime(time()))
                 issue=true;
                 lenToDo=0;
                 apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"location"));
              endif
          else
                 apRobot = setfield(apRobot,"lastRotation",0);           
         endif
         gyroLenToDo=lenToDo;
         robot.Horn(2);  % horn 2 seconds
         pause(5);
         retCode=0;       
         if (lenToDo!=0)
               if (robot.actionRetcode!=0)
                robot.ResetRobotStatus();
                pause(1);
              endif
            [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOff);
            traceRobot=[traceRobot;[time,loopCount,2,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading(),robot.GetGyroHeading(),apGet(apRobot,"subsytemLeft")(1),apGet(apRobot,"subsytemLeft")(2),apGet(apRobot,"subsytemLeft")(3),apGet(apRobot,"subsytemRight")(1),apGet(apRobot,"subsytemRight")(2),apGet(apRobot,"subsytemRight")(3),robot.actionRetcode]];
         else
            apRobot = setfield(apRobot,"lastMove",0);   
         endif
         if (retCode==robot.moveKoDueToNotEnoughSpace && lenToDo!=0 && !issue) % loop decrease move length until move possible
             printf("debug -0\n")
             if (abs(robot.GetRetcodeDetail())>=max(apGet(apRobot,"minDistToBeDone"),lenToDo/2))
                {
                 lenToDo=floor(robot.GetRetcodeDetail())*sign(lenToDo);
                 printf(mfilename);
                 printf(" try new length  x:%d *** ",lenToDo)
                 printf(ctime(time()))
                 printf("debug -3\n");
                [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,plotOn);
                 printf("debug -2\n");
                 traceRobot=[traceRobot;[time,loopCount,3,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading(),robot.GetGyroHeading(),apGet(apRobot,"subsytemLeft")(1),apGet(apRobot,"subsytemLeft")(2),apGet(apRobot,"subsytemLeft")(3),apGet(apRobot,"subsytemRight")(1),apGet(apRobot,"subsytemRight")(2),apGet(apRobot,"subsytemRight")(3),robot.actionRetcode]];
                 printf("debug -1\n");
                 gyroLenToDo=lenToDo; 
                 printf("debug -1.1\n");
                   }           
              else{
                 printf("debug -1.2\n");
                 issue=true;
                 printf(mfilename);
                 apRobot = setfield(apRobot,"lastMove",0);
                 printf(" no possible move max distance:%d *** ",robot.retCodeDetail);
                 printf(ctime(time()));
                 retCode=-98;
                  }
              endif       
         endif
        if (retCode==robot.moveKoDueToNotEnoughSpace && lenToDo!=0 && !issue)
                 printf(mfilename);    
                 printf(" straight move not possible. *** " )
                 printf(ctime(time()))
                 manualMode=true; % temporarly manual mode
                 [apRobot,robot,stopRequested] = ApDetermineCurrentLocation(apRobot,robot,manualMode);
                 if (stopRequested)
                      saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
                      return;
                 endif
        endif
         if (retCode==-99)     % robot timeout
           printf(mfilename);
           printf(" robot no longer communicate ");
           printf(ctime(time())); 
           saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
           return;          
         endif  
         if (retCode==-1)    % action timeout 
           printf(mfilename);
           printf(" move straight timeout");
           printf(ctime(time()));
           if (apGet(apRobot,"saveLocation")(1)==robot.GetHardPosX() && apGet(apRobot,"saveLocation")(2)==robot.GetHardPosY()) % no move
                  apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"saveLocation"));
                  apRobot = setfield(apRobot,"gyroLocation",apGet(apRobot,"saveLocation"));
                  apRobot = setfield(apRobot,"lastMove",0);   
                  apRobot = setfield(apRobot,"particles",apGet(apRobot,"lastParticles"));  % restaure particles
                  printf(mfilename);
                  printf(" hard position not changed >> no move *** ");
                  printf(ctime(time()));
           endif
         endif  
           if (apGet(apRobot,"lastRotation")!=0 || apGet(apRobot,"lastMove")!=0 )
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
                   newHead=[apGet(apRobot,"nextLocation")(3),robot.GetHardHeading(),apGet(apRobot,"gyroLocation")(3),apGet(apRobot,"subsytemLeft")(3),apGet(apRobot,"subsytemRight")(3)];
              else
                   newX=round([apGet(apRobot,"nextLocation")(1),robot.GetHardPosX(),apGet(apRobot,"gyroLocation")(1)]);
                   newY=round([apGet(apRobot,"nextLocation")(2),robot.GetHardPosY(),apGet(apRobot,"gyroLocation")(2)]);
                   newHead=[apGet(apRobot,"nextLocation")(3),robot.GetHardHeading(),apGet(apRobot,"gyroLocation")(3)];
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
               % if greatest distance between 2 points is over 15cm
               %    try to select the best location with sonar
               %
               if ((range(newX)^2+range(newY)^2) > limitForTestLocationEchoConsistancy)         % ecart entre les points les + extremes >15cm
                  printf(mfilename); 
                  printf (" distance between points are too high:%d > try to determine the best with sonar *** ",sqrt(range(newX)^2+range(newY)^2));
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
               else

               printf(mfilename);
               printf(" positions => ");
               for i=1:size(newX,2)
                   printf("%s (%d,%d,%d) ",pointLib(i,:),newX(i),newY(i),newHead(i));
               end  
               printf("\n");
               weightEcho=echoBalance;
               endif
                for i=1:size(newX,2)
                 [available,retCode]=ApQueryCartoAvailability(apRobot,[newX(i),newY(i),newH],degreUnit,debugOn);
                 if (!available)
                    printf(mfilename);
                    printf(" positions problem: (%d,%d) ",newX(i),newY(i));
                    printf(ctime(time()));
                   weightEcho(i)=weightEcho(i)/10;
                 endif
                end  

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
                      saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
                      return;
                   endif
              endif           
                trajectory=[apGet(apRobot,"trajectory");[detX,detY,detH]];          
                apRobot = setfield(apRobot,"trajectory",trajectory);
                trajectory=[trajectory;[detX,detY,detH]];
                prob=round(robot.currentLocProb*.95);
                if (plotValue>=3)
                  plot=true;
                 else
                  plo=false;
                endif
                ApResampleParticles(apRobot,robot,plot);
                [apRobot,robot,retCode] = ApUpdateHardLocation(apRobot,robot,[detX,detY,detH],prob);
                if (retCode==-99)
                    printf(mfilename);
                    printf(" robot no longer communicate ");
                    printf(ctime(time())); 
                    saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,realMode);
                    return;                
                endif
                [apRobot] = ApClearPathStep(apRobot);
           endif
       end
    end
    
   %{
   
   %}
    
    function [] = saveContext(apRobot,traceDet,traceMove,traceNext,traceRobot,traceEcho,horn)
          if (!exist("horn"))  % flat logistic regression is default mode 
            horn=true;
          endif
          if (horn)
           for i=1:5
              robot.Horn(2);  % horn x seconds
              pause(1);
           end
          endif
           cd c:\Users\jean\Documents\Donnees\octave\robot;
           printf(mfilename);    
           printf(" end program *** ");
           printf(ctime(time()))
           save ("-mat4-binary","traceDet.mat","traceDet");
           save ("-mat4-binary","traceMove.mat","traceMove");
           save ("-mat4-binary","traceNext.mat","traceNext");
           save ("-mat4-binary","traceRobot.mat","traceRobot");
           save ("-mat4-binary","traceEcho.mat","traceEcho");
           ApShowStep(apRobot,apGet(apRobot,"trajectory"),"Determined Trajectory");
 %          ApAnalyseTraces();
           return;
    endfunction
  endfunction