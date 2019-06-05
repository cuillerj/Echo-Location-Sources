 function [apRobot,robot,returnLocation,goToRetCode] = ApGotoTarget(apRobot,robot,flatLogRegMode,plotValue)
%{
ApGotoDestination new version
the aim is to give instrucions to the robot to go from the current location to the target location
the robot is supposed to be located with over 50% probability
input:
  the apRobot object provides current and target location
  plotValue is used to plot more or less information depending on the value: 0 means no plot at all
  flatLogRegMode (flat or not) determines the tensor flow mode to use: flat the only currently developped mode

output:
  the location at return time [-1,-1,-1] meaning location issue
  retCode 0 meaning ok else meaning issue
%}
    gotoTargetPathNotFound=-1;
    gotoTargetLocProbError=-2;
    hardIssue=-3;
    locProbRange=apGet(apRobot,"locProbRange");  % probability location range in wich robot is considered as located (under first value = not located between first and second location to be checked
    if (!exist("plotValue"))
      plotValue=0;
    endif
    if (!exist("flatLogRegMode"))
      flatLogRegMode=true;
    endif
    returnLocation=[-1,-1,-1];
    goToRetCode=0;
    cpu1=cputime();
    debugOn=false;     % set to true for debugging
    emergencyStop=false; % flag indicating function stop needed
    loc=apGet(apRobot,"location");  % get the current location of the robot
    locProb=apGet(apRobot,"locationProb"); % get the location probability of the robot (0-100)
    target=apGet(apRobot,"destination");  % get the target location of the robot
    printf(mfilename);
    printf(" robot location is (%d,%d,%d) probability %.1f%% and destination is (%d,%d,%d) *** ",loc(1),loc(2),loc(3),100*locProb,target(1),target(2),target(3));
    printf(ctime(time()));
    [apRobot,robot,robotFigureNumber] = ApDrawRobot(apRobot,robot,1); % draw the robot on the map
    if (locProb<locProbRange(1))
      goToRetCode=gotoTargetLocProbError;
      printf(mfilename);
      printf(" incorrect robot location probability:%d *** ",locProb);
      printf(ctime(time()));
      return;
    endif
    %{
    now create particles filter 
    particles number will increase depending on the incertainty of the location
    particles number will be more or less spreaded depending on the incertainty of the location
    %}
    %{
    sigmaLocation=[10,25];  % sigma range values of the normal distribution
    sigmaHeading=[5,25]; % sigma range values of the normal distribution
    particlesNumber=[1000,3000]; % particles numner range
    probRangeWidth=(locProbRange(3)-locProbRange(2));
    sigmaLoc=sigmaLocation(1)+(sigmaLocation(2)-sigmaLocation(1))*(locProbRange(3)-locProb)/probRangeWidth;
    sigmaHead=sigmaHeading(1)+(sigmaHeading(2)-sigmaHeading(1))*(locProbRange(3)-locProb)/probRangeWidth;
    partNumber=particlesNumber(1)+(particlesNumber(2)-particlesNumber(1))*(locProbRange(3)-locProb)/probRangeWidth;
    apRobot = setfield(apRobot,"locationProb",100); % force probability to for particles creation
    apRobot = ApCreateLocatedParticles(apRobot,partNumber,(plotValue>=2),sigmaLoc,sigmaHead); % create particles for particle filter
    apRobot = setfield(apRobot,"locationProb",locProb);  % restore current probability
    %}

    if (!apGet(apRobot,"autoLocalization"))
       locProb=apGet(apRobot,"locationProb"); % get the location probability of the robot (0-100)
       apRobot = setfield(apRobot,"locationProb",100); % force probability to for particles creation
        apRobot = ApCreateLocatedParticles(apRobot,1000,(plotValue>=2),1.5,0.5); % robot manualy located - set noise at minimum level
        apRobot = setfield(apRobot,"locationProb",locProb);  % restore current probability
    else
       apRobot = ApCreateParticlesForGoToTarget(apRobot,(plotValue>=2));
    endif
    
    %{
    initialize octave data 
    %}
  %  distanceToTargetMargin=15; % cm
    radiusMargin=apGet(apRobot,"radiusMargin"); % cm
    headingMargin=apGet(apRobot,"headingMargin"); % °

    zonesXY=apGet(apRobot,"scanRefPoints");       % list of each point learnt by the AI 
    spaceNO=SpaceNorthOrientation(zonesXY,loc(1),loc(2));  % get the north orientation of the current location based on the closest known point
    issue=0;                    % flag used to identify any issues
    emergencyStop=false;        % flag used to stop hard robot
    targetReached=false;        % flag used to determine robot has reach the target location
    ready=false;                % flag used to determine if the hard robot is ready to work or not
    shiftNO=apGet(apRobot,"currentShiftNorthOrientation");
    simulationMode=apGet(apRobot,"simulationMode");
    realMode=apGet(apRobot,"realMode"); 
    apRobot = setfield(apRobot,"newTarget",true);
     checkLocationAvaibility=true;
    %{
    initialize java data 
    %}
    robot.SetPosX(loc(1));
    robot.SetPosY(loc(2));
    robot.SetHeading(loc(3));
    robot.SetCurrentLocProb(locProb);
    %{
    initialize automaton
    %}
  [mStatus,lStatus,aStatus] = ApAutomatonStatusList(apRobot);
  initial=apGet(apRobot,"automatonInitial");  
  localizing=apGet(apRobot,"automatonLocalizing"); 
  targeting=apGet(apRobot,"automatonTargeting");
  gotTarget=apGet(apRobot,"automatonGotTarget");
  locked=apGet(apRobot,"automatonLocked");
  lost=apGet(apRobot,"automatonLost");

  %mStatus={"initial";"localizing";"targeting";"gotTarget";"locked";"lost"};
   %{
  localization status
  %} 
  notLocalized=apGet(apRobot,"automatonNotLocalized");
  localized=apGet(apRobot,"automatonLocalized");
  localisationLost=apGet(apRobot,"automatonLocalisationLost");
  determining=apGet(apRobot,"automatonDetermining");
  %lStatus={"notLocalized";"localized";"localisationLost";"determining"};
  %{
  action status
  %}
  atRest=apGet(apRobot,"automatonAtRest");
  NOrient=apGet(apRobot,"automatonNOrient");
  moving=apGet(apRobot,"automatonMoving");
  scanned=apGet(apRobot,"automatonScanned");
  
  
  moveStraight=apGet(apRobot,"automatonMoveStraight");
  rotate=apGet(apRobot,"automatonRotate");
  northAlign=apGet(apRobot,"automatonNorthAlign");
  scan360=apGet(apRobot,"automatonScan360");
  determine=apGet(apRobot,"automatonDetermine");
  pingFB=apGet(apRobot,"automatonPingFB");
  checkTarget=apGet(apRobot,"automatonCheckTarget");
  checkLocation=apGet(apRobot,"automatonCheckLocation");
  acrossPath=apGet(apRobot,"automatonAcrossPath");
  actionList = apGet(apRobot,"automatonActionList");
  %{
  
    initial=1;
    localizing=2;
    targeting=3;
    gotTarget=4;
    locked=5;
    lost=6;
    [mStatus,lStatus,aStatus] = ApAutomatonStatusList(apRobot);
    %mStatus={"initial";"localizing";"targeting";"gotTarget";"locked";"lost"};
     %{
    localization status
    %} 
    notLocalized=1;
    localized=2;
    localisationLost=3;
    determining=4;
   % lStatus={"notLocalized";"localized";"localisationLost";"determining"};
    %{
    action status
    %}
    atRest=1;
    NOrient=2;
    moving=3;
    scanned=4;
    %aStatus={"atRest";"NOrient";"moving";"scanned";"scanned"};

    % actions
    determine=5;
    checkTarget=7;
    %}
    %{
    initialize hard location 
    %}
    automaton=apGet(apRobot,"automatonState");
    rotationType=2;  % select 1="northOrientation" 2="gyroscope" 3="wheels"
    printf(mfilename);
    printf(" starting automaton status is (%s,%s,%s) *** ",char(mStatus(automaton(1))),char(lStatus(automaton(2))),char(aStatus(automaton(3))));
    printf(ctime(time()));
    printf(mfilename);
    printf(" update hard robot. *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
    printf(ctime(time()))
    while((robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360)) && !emergencyStop)
        robot.UpdateHardRobotLocation();	
        apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);
          [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,0);
          if (retCode!=0)
            [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
            if (issue)
              emergencyStop=true;
              goToRetCode=hardIssue;
              break
             endif
          endif
    end

    %{
    noise North orientation adjustment
    %}
    if (!realMode)
      NO=normrnd(spaceNO,5);  % simulate noise NO
      headingNO=mod(360+spaceNO-NO,360);   % determine heading regarding magneto mesurment and value for echo scan reference
      shiftNO=spaceNO-NO;
    else
      NO=robot.northOrientation;          %  request NO info
      apRobot = setfield(apRobot,"waitFor",robot.robotInfoUpdated); 
      [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
      if (retCode!=0)
        [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
        if (issue)
          emergencyStop=true;
          goToRetCode=hardIssue;
        endif
        NO=robot.northOrientation;           % get the up to date info
        eadingNO=mod(360+spaceNO-NO,360); 
        shiftNO=spaceNO-NO;
      endif
    endif
    printf(mfilename);
    printf(" robot shiftNO:%d *** ",spaceNO);
    printf(ctime(time())); 
    loopCount=0;
    apRobot=setfield(apRobot,"traceNext",[apGet(apRobot,"traceNext");[time,loopCount,loc(1),loc(2)]]);
    
 %   while (apGet(apRobot,"automatonState")(1)==targeting && apGet(apRobot,"automatonState")(2)== localized && apGet(apRobot,"automatonState")(2)!= localisationLost)
     while ( apGet(apRobot,"automatonState")(2)== localized)
      % is the target reached ?
      printf(mfilename);
      apRobot = setfield(apRobot,"location",[apGet(apRobot,"location")(1),apGet(apRobot,"location")(2),mod(apGet(apRobot,"location")(3),360)]);
      printf(" loop %d: location is (%d,%d,%d) *** ",loopCount,apGet(apRobot,"location")(1),apGet(apRobot,"location")(2),apGet(apRobot,"location")(3));
      printf(ctime(time()));
      [apRobot,robot,robotFigureNumber] = ApDrawRobot(apRobot,robot,0,robotFigureNumber); % draw the robot on the map
      apRobot=setfield(apRobot,"trajectory",[apGet(apRobot,"trajectory");apGet(apRobot,"location")(1:2)]);
 %     distToTarget=sqrt((target(1)-robot.GetHardPosX)^2 + (target(2)-robot.GetHardPosY)^2);
      [targetReached,remainingDistance] = ApTargetReached(apRobot,robot);
      if (targetReached)
          apRobot = setfield(apRobot,"automatonState",[gotTarget,apGet(apRobot,"automatonState")(2),apGet(apRobot,"automatonState")(3)]);
          returnLocation=apGet(apRobot,"location");
          apRobot = setfield(apRobot,"location",returnLocation);
          goToRetCode=0;
          printf(mfilename);
          printf(" target reached ! (distance to target:%d) *** ",remainingDistance);
          printf(ctime(time()));        
          robot.Horn(3);
          break;
      endif
      pause(5)
      loopCount++; 
      [available,retCode] = ApQueryCartoAvailability(apRobot,loc,false,true);
      if(!available)
          apRobot = setfield(apRobot,"automatonState",[initial,localisationLost,atRest]); 
          printf(mfilename);
          printf(" location theoriticaly not possible  *** ");
          printf(ctime(time()));
          goToRetCode=0;
          break;    
      endif
      switch(apGet(apRobot,"automatonState"))
        case([targeting,localized,atRest])
#{
          if (size(apGet(apRobot,"narrowPath"),1)>1) % is there a narrow path on the way ?
              [pathFound,pathNumber,entryPoint,pathHeading,pathDistance,farDistances] = ApLookForForNarrowPath(apRobot,robot,100);
              if (pathFound)
                  printf(mfilename);
                  printf(" robot close to a narrow path:%d heading:%d distance:%d *** ",pathNumber,pathHeading,pathDistance);
                  printf(ctime(time()));
                 if (pathNumber==apGet(apRobot,"narrowPath")(2))
                    farther=((apGet(apRobot,"destination")(1)^2+apGet(apRobot,"destination")(2)^2) > (apGet(apRobot,"location")(1)^2+apGet(apRobot,"location")(2)^2));
                    [apRobot,robot,retCode] = ApGoThruNarrowPath(apRobot,robot,pathNumber,entryPoint,pathHeading,pathDistance,farDistances,farther,forward);
                   % break;
                 endif
             endif
          endif
 #}
          [apRobot,robot,probability] = ApCompareParticlesAndLocation(apRobot,robot,radiusMargin,headingMargin);
          if (probability < locProbRange(1))
             printf(mfilename);
             printf(" location probability too low:%d *** ",100*probability)
             printf(ctime(time()))
             break;
          elseif (probability < locProbRange(2))

             printf(mfilename);
             printf(" probability low > need to check location:%.1f%% *** ",100*probability);
             printf(ctime(time()));
             saveLocation=apGet(apRobot,"location");
             saveProb=apGet(apRobot,"locationProb");
             [apRobot,robot,TfX,TfY,TfH,TfProb,retCode] = ApEchoLocalizeRobotWithTensorFlowRotated(apRobot,robot,(plotValue>=2));
             [apRobot,robot,detX,detY,detH,prob,figureNumber] = ApDetermineRobotLocationWithTfAndParticlesGaussian(apRobot,robot,TfX,TfY,TfProb,(plotValue>=1),1);
             apRobot = setfield(apRobot,"location",[detX,detY,detH]);
  %           apRobot = setfield(apRobot,"locationProb",min(apGet(apRobot,"locationProb"),probability));
             apRobot = setfield(apRobot,"locationProb",probability);
             [apRobot,robot] = ApResampleParticles(apRobot,robot,(plotValue>=3),true,checkLocationAvaibility);
             [apRobot,robot,probability] = ApCompareParticlesAndLocation(apRobot,robot,radiusMargin,headingMargin);
             if (probability<apGet(apRobot,"locProbRange")(1))
                 TfLocation=[TfX(1),TfY(1)];
                 [located,consistant,distance,deltaHeading,bestLocation] = ApDetermineRobotLocationLostOrNot(apRobot,saveLocation,saveProb,TfLocation,TfProb(1));
                  printf(mfilename);
                  printf(" Check location entry:(%d,%d,%d - %.1f%%) determined: (%d,%d,%d - %.1f%%) Tf: (%d,%d) - %.1f%%) *** ",saveLocation(1),saveLocation(2),saveLocation(3),100*saveProb,detX,detY,detH,100*probability,TfX(1),TfY(1),100*TfProb(1));
                  printf(ctime(time()))
                 if ((located) && (consistant))
                   printf(mfilename);
                   printf(" confirmed location - distance:%d deltaHeading:%d probability:%.1f%% *** ",distance,deltaHeading,100*probability);
                   printf(ctime(time()))
                 elseif ((!located) && (!consistant))              
                   printf(mfilename);
                   printf(" Location lost - distance:%d deltaHeading:%d probability:%.1f%% *** ",distance,deltaHeading,100*probability)
                   printf(ctime(time()))
                   apRobot = setfield(apRobot,"location",bestLocation);                        % defined / determined location 
                   apRobot = setfield(apRobot,"locationProb",0);
                   apRobot = setfield(apRobot,"automatonState",[initial,notLocalized,atRest]);
                   apRobot = setfield(apRobot,"newTarget",true);
                   break;   
                 elseif ((located) && (!consistant))
                    printf(mfilename);
                    printf(" new determined location is likely right *** ")
                    printf(ctime(time()))
                    apRobot = setfield(apRobot,"location",bestLocation);                        % defined / determined location 
                    apRobot = setfield(apRobot,"locationProb",min(saveProb,probability));   
                    [apRobot,robot] = ApUpdateHardLocation(apRobot,robot,apGet(apRobot,"location"),apGet(apRobot,"locationProb"));
                    apRobot = ApCreateParticlesForGoToTarget(apRobot,(plotValue>=1));
                    apRobot = setfield(apRobot,"newTarget",true);   
                 elseif (!located && consistant)
                    printf(mfilename);
                    printf(" new determined location close to the estimated one *** ")
                    printf(ctime(time()))
                    if (saveProb>probability)
                      apRobot = setfield(apRobot,"location",bestLocation);                        % defined / determined location   
                      apRobot = setfield(apRobot,"locationProb",min(saveProb,probability));
                      [apRobot,robot] = ApUpdateHardLocation(apRobot,robot,apGet(apRobot,"location"),apGet(apRobot,"locationProb"));  
                      apRobot = ApCreateParticlesForGoToTarget(apRobot,(plotValue>=1));
                      apRobot = setfield(apRobot,"newTarget",true);
                    endif   
                 endif
             endif  
          endif  
          if(apGet(apRobot,"newTarget")); % do we need to compute new path ?
            [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,target,(plotValue>=2));
          endif
          [apRobot,robot,next,direct,forward] = ApComputeNextStepToTarget(apRobot,robot,(plotValue>=2));
  %        newTarget=0;
          if (forward==-9)       
             printf(mfilename);
             printf(" no path found. *** ")
             printf(ctime(time()))
             goToRetCode=gotoTargetLocProbError;
             break;     
          endif
          printf(mfilename);
          printf(" Next step is n°:%d X:%d Y:%d . *** ",loopCount,next(1),next(2))
          printf(ctime(time()))
          apRobot=setfield(apRobot,"traceNext",[apGet(apRobot,"traceNext");[time,loopCount,next]]);
          [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward);
          saveLocation=[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading];
          printf(mfilename);
          printf(" rotation:%d distance:%d *** ",rotationToDo,lenToDo);
          printf(ctime(time()));
          saveTargetHeading=robot.GetHeading()+rotationToDo;  % replace rotation by rotationToDo le 28/09
          saveHeading=robot.GetHeading();
          apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,rotationToDo,0]]);
          [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,(plotValue>=4));
          apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
          apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,1,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCode]]);
          if (action=="stop..")
              printf(mfilename);
              printf(" stop due to pb rotation")
              printf(" *** ");
              issue==true;
              apRobot = setfield(apRobot,"automatonState",[lost,apGet(apRobot,"automatonState")(2),apGet(apRobot,"automatonState")(3)]); 
              goToRetCode=hardIssue;
              break
          endif
          
        case([targeting,localized,moving])
              apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,0,lenToDo]]);
              [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,(plotValue>=2));
              apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
              apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,2,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCode]]);
        case([locked,localized,atRest])  
              printf(mfilename);
              printf(" locked state:(%d,%d,%d) *** ",apGet(apRobot,"automatonState")(1),apGet(apRobot,"automatonState")(2),apGet(apRobot,"automatonState")(3));
              printf(ctime(time())); 
              randomChoice=true;
              retCodeRetry=-1;
          %    apGet(apRobot,"currentMove")
            %  apGet(apRobot,"retcodeDetail")
              [idxR,j]=find(flipud(apGet(apRobot,"traceRobot"))(:,3)==1,1) ; %  look for last rotate retCode
              [idxS,j]=find(flipud(apGet(apRobot,"traceRobot"))(:,3)==2,1)  %  loook for last move straight retCode
              if (size(idxR,1)==1 && ((apGet(apRobot,"traceRobot"))(idxR,1)>(apGet(apRobot,"traceRobot"))(idxS,1))) % if last non null retcode is rotation
                  flipud(apGet(apRobot,"traceRobot"))(idxR,8)
                 if (flipud(apGet(apRobot,"traceRobot"))(idxR,8)==robot.moveKoDueToNotEnoughSpace && apGet(apRobot,"currentMove")=="rotate" && apGet(apRobot,"retcodeDetail")>=3*apGet(apRobot,"minRotToBeDone"))
                    rotationToDo=round(apGet(apRobot,"lastRotation")*.5);
                    printf(mfilename);
                    printf(" rerty rotate with rotationToDo:% d *** ",rotationToDo);
                    printf(ctime(time()));
                    apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,rotationToDo,0]]);
                    [apRobot,robot,retCodeRetry,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,(plotValue>=3));
                    apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
                    apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,1,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeRetry]]);
                  endif
              endif
              [idxS,j]=find(flipud(apGet(apRobot,"traceRobot"))(:,3)==2,1)  %  loook for last move straight retCode
              if (size(idxS,1)==1 && ((apGet(apRobot,"traceRobot"))(idxS,1)>=(apGet(apRobot,"traceRobot"))(idxR,1))) % if last non null retcode is move straight
                flipud(apGet(apRobot,"traceRobot"))(idxS,8)
                if (flipud(apGet(apRobot,"traceRobot"))(idxS,8)==robot.moveKoDueToNotEnoughSpace && apGet(apRobot,"currentMove")=="straig" && apGet(apRobot,"retcodeDetail")>=3*apGet(apRobot,"minDistToBeDone"))
                    maxLen=round(robot.GetRetcodeDetail()*.7);
                    printf(mfilename);
                    printf(" rerty move straight with len:% d *** ",maxLen);
                    printf(ctime(time()));
                    apRobot=setfield(apRobot,"traceMove",[apGet(apRobot,"traceMove");[time,loopCount,0,maxLen]]);
                    [apRobot,robot,retCodeRetry]=ApRobotMoveStraight(apRobot,robot,maxLen,forward,(plotValue>=2));
                    apRobot = setfield(apRobot,"location",[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading]);
                    apRobot = setfield(apRobot,"traceRobot",[apGet(apRobot,"traceRobot");[time,loopCount,2,[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading],robot.GetGyroHeading(),retCodeRetry]]);
                endif
              endif
              if (retCodeRetry==0)
                   apRobot = setfield(apRobot,"automatonState",[targeting,localized,atRest]);  % 
              else
                 [apRobot,robot,unlocked,retCode] = ApUnlockRobot(apRobot,robot,loopCount);
                if (unlocked==true)
                 apRobot = setfield(apRobot,"automatonState",[targeting,localized,atRest]);  % 
                else
                  apRobot = setfield(apRobot,"automatonState",[lost,apGet(apRobot,"automatonState")(2),apGet(apRobot,"automatonState")(3)]);
                  locRetCode=-1;
                return
             endif
            endif    
        otherwise       
          break;
      endswitch     
    endwhile

 % AStarShowStep(trajectory,"determined trajectory");
    ApAStarShowStep(apRobot,apGet(apRobot,"trajectory"),"determined trajectory"); 
    traceMove=apGet(apRobot,"traceMove");
    traceNext=apGet(apRobot,"traceNext");
    traceRobot=apGet(apRobot,"traceRobot");
    traceEcho=apGet(apRobot,"traceEcho");
    save ("-mat4-binary","traceMove.mat","traceMove");
    save ("-mat4-binary","traceNext.mat","traceNext");
    save ("-mat4-binary","traceRobot.mat","traceRobot");
    save ("-mat4-binary","traceEcho.mat","traceEcho");
    t=clock();
    fname=strcat("moves",mat2str(t(1)),mat2str(t(2)),mat2str(t(3)),mat2str(t(4)),mat2str(t(5)),mat2str(round(t(6))),".jpg");
    saveas(robotFigureNumber,fname,"jpg");
    hold off
    printf(mfilename);
    printf(" exit automaton status is (%s,%s,%s) *** ",char(mStatus(apGet(apRobot,"automatonState")(1))),char(lStatus(apGet(apRobot,"automatonState")(2))),char(aStatus(apGet(apRobot,"automatonState")(3))));
    printf(ctime(time()));
    loc=apGet(apRobot,"location"); % get the location probability of the robot (0-100)
    printf(mfilename);
    printf(" exit robot location is (%d,%d,%d) probability %.1f%% and destination is (%d,%d,%d) *** ",loc(1),loc(2),loc(3),apGet(apRobot,"locationProb")*100,target(1),target(2),target(3));
    printf(ctime(time()));
    [apRobot,robot,robotFigureNumber] = ApDrawRobot(apRobot,robot,0,robotFigureNumber); % draw the robot on the map
    robot.Horn(5);  % horn 
  return;
 
 endfunction