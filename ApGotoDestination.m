 function [apRobot,robot,returnLocation] = ApGotoDestination(apRobot,robot,flatLogRegMode,plotValue)
%{

%}
  if (!exist("plotValue"))
    plotValue=0;
  endif
  returnLocation=[-1,-1,-1];
  cpu1=cputime();
  debugOn=false;
  flat=flatLogRegMode;
  emergencyStop=false;
  loc=apGet(apRobot,"location");
  locProb=apGet(apRobot,"locationProb");
  newProb=locProb;
  newX=loc(1);
  newY=loc(2);
  newH=loc(3);
  target=apGet(apRobot,"destination");
  printf(mfilename);
  printf(" robot location is (%d,%d,%d) and destination is (%d,%d,%d) *** ",loc(1),loc(2),loc(3),target(1),target(2),target(3));
  printf(ctime(time()));
  sigmaPos=15;
  sigmaHeading=10;
  particlesNumber=1000;
  [apRobot] = ApCreateLocatedParticles(apRobot,particlesNumber,(plotValue>=1),sigmaPos,sigmaHeading);
  [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,target,(plotValue>=1));
  zonesXY=apGet(apRobot,"scanRefPoints");
  spaceNO=SpaceNorthOrientation(zonesXY,loc(1),loc(2));  % 
  trajectory=[loc(1),loc(2)];
  traceDet=[];
  traceMove=[];
  traceNext=[];
  traceRobot=[];
  traceEcho=[];
  robot.SetPosX(loc(1));
  robot.SetPosY(loc(2));
  robot.SetHeading(loc(3));
  robot.SetCurrentLocProb(locProb);
  all_theta=apGet(apRobot,"all_theta");
  nbLocPossibility=size(all_theta,1); % determine the number of zones used during the trainning phase
  predLocMatrix=InitLocMatrix(all_theta);
  j=1;
  issue=0;                    % flag used to identify any issues
  targetReached=false;        
  ready=false;
  nbPred=5;                    % define the number of predictions that will be compute for each 360° scan
  rotationType=2;  % select 1="northOrientation" 2="gyroscope" 3="wheels"
  probExpectedMoveOk=50;
  probHardMoveOk=50;
  pingFB=6;
  checkTarget=7;
  %{
  main status
  %}
  initial=1;
  localizing=2;
  targeting=3;
  gotTarget=4;
  locked=5;
  lost=6;
  
  shiftNO=apGet(apRobot,"currentShiftNorthOrientation");
  simulationMode=apGet(apRobot,"simulationMode");
  realMode=apGet(apRobot,"realMode"); 
   if (simulationMode==0)
    echoBalance=[1,0.95,1.1,1.1];  % theoritical , encoder  , BNO Left, BNO Right
  else
    echoBalance=[1,0.95,1.05];  % theoritical , encoder , gyroscope
  endif
  gyroBasedX=loc(1);
  gyroBasedY=loc(2);
  gyroBasedH=loc(3)*pi()/180;
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
          break
         endif
      endif
  end
   if (simulationMode!=0)
    NO=normrnd(spaceNO,5);  % simulate noise NO
    headingNO=mod(360+spaceNO-NO,360);   % determine heading regarding magneto mesurment and value for echo scan reference
    shiftNO=spaceNO-NO;
  else
    NO=robot.northOrientation;          %  request NO info
 %   WaitFor=WaitInfo;
   apRobot = setfield(apRobot,"waitFor",robot.robotInfoUpdated); 
   [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
    if (retCode!=0)
      [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
      if (issue)
        emergencyStop=true;
      endif
      NO=robot.northOrientation;           % get the up to date info
      eadingNO=mod(360+spaceNO-NO,360); 
      shiftNO=spaceNO-NO;
    endif
  endif
  posX=loc(1);
  posY=loc(2);
  locProb=newProb;
  heading=loc(3);
  retCode=99;
  %robot.northOrientation=NO;  
  printf(mfilename);
  printf(" robot shiftNO:%d *** ",spaceNO);
  printf(ctime(time()));
  %{
      compute target location
  %}
  %[targetX,targetY,targetAngle]=ComputeTargetLocation(carto,robot);

  %apRobot = setfield(apRobot,"destination",[targetX,targetY,targetH]); 
  apRobot = setfield(apRobot,"newTarget",true);  
  targetX=target(1);
  targetY=target(2);
  targetH=target(3);
  printf(mfilename);
  printf(" robot target is X:%d Y:%d orientation: %d. *** ",targetX,targetY,targetH)
  printf(ctime(time()))
  %
  [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,[targetX,targetY,targetH],(plotValue>=1));
  robot.SetPosX(posX);
  robot.SetPosY(posY);
  robot.SetHeading(heading);
  robot.SetCurrentLocProb(locProb);
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
          break
         endif
      endif
  end
  if (simulationMode==0)  
          robot.SetPosX(round(newX));
          robot.SetPosY(round(newY));
          robot.SetHeading(round(newH));
          robot.SetCurrentLocProb(newProb);
          printf(mfilename);
          printf(" update hard robot.. *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading());
          printf(ctime(time()));
          while((robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360)) && !emergencyStop)
            robot.UpdateHardRobotLocation();
            apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);
            [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
            if (retCode!=0)
              [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
               if(issue)
                  emergencyStop=true;
                  break
               endif
            endif
          end
          printf(mfilename);
          printf(" hardPosX:%d hardPosY:%d *** ",robot.GetHardPosX,robot.GetHardPosY);
          printf(ctime(time()))
  endif
  loopCount=0;	
  traceNext=[traceNext;[time,loopCount,posX,posY]];
  gyroBasedX=gyroBasedX-apGet(apRobot,"shiftEchoVsRotationCenter")*cos(newH);           % set position to rotation center
  gyroBasedY=gyroBasedY-apGet(apRobot,"shiftEchoVsRotationCenter")*sin(newH);           % set position to rotation center

  while (apGet(apRobot,"automatonState")(1)==targeting)
    pause(5)
    loopCount++;
    rotation=0;
        if ((abs(robot.GetHardPosX-round(newX))<=1  && abs(robot.GetHardPosY-round(newY))<=1 ))
              if (ApQueryCartoAvailability(apRobot,[newX,newY,newH],false,1))
                [available,retCode] = ApQueryCartoAvailability(apRobot,[newX,newY,newH],false,true);
                currentPositionIssue=true;
              endif
              if ((targetX-robot.GetHardPosX)^2 + (targetY-robot.GetHardPosY)^2 <=225)
                targetReached=true;
                [apRobot,robot,newState,rc] = ApAutomaton(apRobot,robot,[checkTarget,true],1);
                returnLocation=[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading];
                % check target reached
              else 
                % compute trajectory step
                if (simulationMode==0)
                  robot.Horn(3);  % horn 20 seconds
                  pause(3);
  %							inp=input("take a picture ");
                endif
                [apRobot,robot,next,direct,forward] = ApComputeNextStepToTarget(apRobot,robot,(plotValue>=1));
                newTarget=0;
                if (forward==-9)
                  printf(mfilename);
                  printf(" no path found. *** ")
                  printf(ctime(time()))
                  issue=true;
                  
                  AStarShowStep(trajectory,"determined trajectory");
                  save ("-mat4-binary","traceDet.mat","traceDet");
                  save ("-mat4-binary","traceMove.mat","traceMove");
                  save ("-mat4-binary","traceNext.mat","traceNext");
                  save ("-mat4-binary","traceRobot.mat","traceRobot");
                  save ("-mat4-binary","traceEcho.mat","traceEcho");
                  return
                endif
                printf(mfilename);
                printf(" Next step is n°:%d X:%d Y:%d . *** ",loopCount,next(1),next(2))
                printf(ctime(time()))
                traceNext=[traceNext;[time,loopCount,next]];
  %							if (direct==false)
                [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward);
                saveLocation=[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading];
                traceMove=[traceMove;[time,loopCount,rotationToDo,lenToDo]];
                printf(mfilename);
                printf(" rotation:%d distance:%d *** ",rotationToDo,lenToDo);
                printf(ctime(time()));
                saveTargetHeading=robot.GetHeading()+rotationToDo;  % replace rotation by rotationToDo le 28/09
                saveHeading=robot.GetHeading();
%                 if (simulationMode==1)
    %							rotationToDo=rotationParticlesToDo;
  %              endif
                if (rotationToDo!=0)
				           robot.Horn(1);  % horn x seconds
                   [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,(plotValue>=3));
                   traceRobot=[traceRobot;[time,loopCount,1,robot.GetHardPosX(),robot.GetHardPosY(),+robot.GetHardHeading(),robot.GetGyroHeading()],retCode];
                   %{
                  if (retCode==robot.moveKoDueToSpeedInconsistancy)
                      printf("rotate back to try to remove from obstacle:%d ",-sign(rotationToDo)*minRotGyroAbility)
                      printf(" *** ");
                      robotRotate(-sign(rotationToDo)*minRotGyroAbility);      % eventualy try to remove from obstacle
                      pause(5);
                   endif
                   %}
                   
                   if (action=="stop..")
                     printf(mfilename);
                     printf(" stop due to pb rotation")
                     printf(" *** ");
                     issue==true;
                     lenToDo=0;
                   endif

                endif
                gyroLenToDo=lenToDo;
%                if (simulationMode==1)
    %							lentoDo=lenParticlesToDo; 		 % len sent in cm
  %              endif
                robot.Horn(2);  % horn 2 seconds
                pause(5);
                if (lenToDo!=0&&retCode==0)
  %                retCode=robotMoveStraight(lenToDo);
                  [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,(plotValue>=1));
                  traceRobot=[traceRobot;[time,loopCount,1,robot.GetHardPosX(),robot.GetHardPosY(),+robot.GetHardHeading(),robot.GetGyroHeading()],retCode];
                endif
                moveRetry=0;
                retCodeMove=retCode;
                while (retCodeMove==robot.moveKoDueToNotEnoughSpace)
                  if (robot.retCodeDetail>=apGet(apRobot,"minDistToBeDone") && moveRetry <4)
                    {
                    lenToDo=robot.retCodeDetail-5;
                    printf(mfilename);
                    printf(" try new length  x:%d *** ",lenToDo)
                    printf(ctime(time()))
                    [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,(plotValue>=1));
                    gyroLenToDo=lenToDo; 
                    next(1)=round(apGet(apRobot,"saveLocation")(1)+cos((apGet(apRobot,"saveLocation")(3)+rotationToDo)*pi()/180)*lenToDo)
                    next(2)=round(apGet(apRobot,"saveLocation")(2)+sin((apGet(apRobot,"saveLocation")(3)+rotationToDo)*pi()/180)*lenToDo)
                    moveRetry++;
                    }           
                  else{
                    issue=true;
                    printf(mfilename);
                    printf(" no possible move to many retry :%d or max distance:%d *** ",moveRetry,robot.retCodeDetail);
                    printf(ctime(time()));
                    automatonState=apGet(apRobot,"automatonState");
                    apRobot = setfield(apRobot,"automatonState",[locked,automatonState(2),automatonState(3)]);  
                    retCode=99;
                   }
                  endif       
                end
                if (retCodeMove==robot.moveUnderLimitation)
                    [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,[targetX,targetY,targetH],(plotValue>=1));
                endif
                  %{
                  if (retCode==robot.moveKoDueToSpeedInconsistancy)
                    printf("move back to try to remove from obstacle: %d,-sign(lenToDo)*minDistToBeDone ")
                    printf(" *** ");
                    robotMoveStraight(-sign(lenToDo)*minDistToBeDone);      % eventualy try to remove from obstacle
                    pause(5);
                  endif
                  %}
                  subsystemLeftX=robot.BNOLeftPosX;
                  subsystemLeftY=robot.BNOLeftPosY;
                  subsystemRightX=robot.BNORightPosX;
                  subsystemRightY=robot.BNORightPosY;
                  subsystemHeading=robot.BNOLocHeading;
                  printf(mfilename);
                  printf(" Subsystem location leftX:%d leftY:%d rightX:%d rightY:%d heading:%d",subsystemLeftX,subsystemLeftY,subsystemRightX,subsystemRightY,subsystemHeading);
                  printf(" *** ");
                  printf(ctime(time()));
                  gyroBasedH=mod(gyroBasedH+robot.GetGyroHeading()*pi()/180,2*pi());
                  gyroBasedX=round(gyroBasedX+gyroLenToDo*cos(gyroBasedH)+apGet(apRobot,"shiftEchoVsRotationCenter")*cos(gyroBasedH));
                  gyroBasedY=round(gyroBasedY+gyroLenToDo*sin(gyroBasedH)+apGet(apRobot,"shiftEchoVsRotationCenter")*sin(gyroBasedH));
                  if (simulationMode==0)
                    newX=[next(1),robot.GetHardPosX(),subsystemLeftX,subsystemRightX];
                    newY=[next(2),robot.GetHardPosY(),subsystemLeftY,subsystemRightY];
                    %newX=[next(1),robot.GetHardPosX(),gyroBasedX,subsystemLeftX,subsystemRightX];
                    %newY=[next(2),robot.GetHardPosY(),gyroBasedY,subsystemLeftY,subsystemRightY];
                  else
                    newX=[next(1),robot.GetHardPosX(),gyroBasedX];
                    newY=[next(2),robot.GetHardPosY(),gyroBasedY];
                  endif
                  gyroBasedX=gyroBasedX-apGet(apRobot,"shiftEchoVsRotationCenter")*cos(gyroBasedH); % set position to rotation center
                  gyroBasedY=gyroBasedY-apGet(apRobot,"shiftEchoVsRotationCenter")*sin(gyroBasedH); % set position to rotation center
                  apRobot = setfield(apRobot,"gyroLocation",[gyroBasedX,gyroBasedY,gyroBasedH]);
                  if (rotationType==2)
                    newH=[mod(360+gyroBasedH*180/pi(),360)];
                  else
                    newH=[mod(360+robot.GetHardHeading(),360)];
                  endif
                  [available,retCode] = ApQueryCartoAvailability(apRobot,[robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading()],false,1);
                  if(available==false)
                    probHardMoveOk=1;  % the hard position is theoriticaly not possible
                    printf(mfilename);
                    printf(" the hard position is theoriticaly not possible. *** x:%d y:%d h:%d ",robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading())
                    printf(ctime(time()))
                    automatonState=apGet(apRobot,"automatonState");
                    apRobot = setfield(apRobot,"automatonState",[locked,automatonState(2),automatonState(3)]);  
                  endif
                  newProb=[probExpectedMoveOk,probHardMoveOk];
                  if (probExpectedMoveOk >= probHardMoveOk)
                    spaceNO=SpaceNorthOrientation(zonesXY,newX(1),newY(1))-shiftNO-robot.northOrientation;  
                  else
                    spaceNO=SpaceNorthOrientation(zonesXY,newX(2),newY(2))-shiftNO-robot.northOrientation;
                  endif
                  headingNO=mod(360+spaceNO+shiftNO,360); 
    %							prob=newProb;
                  [line,col]=size(newX);
                  weightEcho=[];
                  if (rotationType==2)
                    averageH=gyroBasedH;
                  else
                    alpha1=newH*pi()/180;
                    alpha3=saveTargetHeading*pi()/180;
                    if (simulationMode==1)
                      averageH=atan2((sin(alpha1)+sin(alpha3)),(cos(alpha1)+cos(alpha3)));
                    else
                      alpha2=headingNO*pi()/180;
                      averageH=mod(atan2((sin(alpha1)+sin(alpha2)+sin(alpha3)),(cos(alpha1)+cos(alpha2)+cos(alpha3)))+2*pi(),2*pi());
                    endif
                  endif
                  printf(mfilename);
                  printf (" Heading hard H:%d  NO:%d Theoretical H:%d  Average:%f SpaceNO:%d RobotNO:%d Gyro:%d*** ",newH,headingNO,saveTargetHeading,averageH*180/pi(),spaceNO,robot.northOrientation,robot.GetGyroHeading());
                  printf(ctime(time()))
                  if (std(posX)>10 || std(posY)>10)
                      [apRobot,robot,weightEcho,retValue,echo,quality] = ApTestLocationEchoConsistancyVsDB(apRobot,robot,newX,newY,averageH);
                      [apRobot,robot,newState,rc] = ApAutomaton(apRobot,robot,[pingFB,retValue],1);
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
                    [nbRow,nbCol]=size(weightEcho);
                    printf(mfilename);
                    printf(" best quality (one is perfect):%f",quality);
                    printf(" *** ");
                    printf(ctime(time()));
                    weightEcho=weightEcho.*echoBalance;   % 
                 else
                  weightEcho=echoBalance;
                endif
                [apRobot,robot,detX,detY,detH] = ApDetermineRobotLocationWithParticlesGaussian(apRobot,robot,newX,newY,weightEcho,(plotValue>=1),1);
                traceDet=[traceDet;[time,loopCount,detX,detY,detH]];
                printf(mfilename);
                printf(" determined X: %f ",round(detX))
                printf(" Y: %f ",round(detY))
                printf(" H: %f ",round(detH))
                printf(ctime(time()))
                trajectory=[trajectory;[detX,detY]];
                newX=detX;
                newY=detY;
                newH=detH;
                robot.SetPosX(round(detX));
                robot.SetPosY(round(detY));
                robot.SetHeading(round(detH));
                apRobot = setfield(apRobot,"location",[detX,detY,detH]);
  %							printf("det prob: %d *** ",prob);
  %							printf(ctime(time()));
  %							robot.SetCurrentLocProb(prob(1));
                printf(mfilename);
                printf(" update hard robot... *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
                printf(ctime(time()))
                while(robot.GetPosX()!=robot.GetHardPosX() || mod(robot.GetPosY(),360)!=mod(robot.GetHardPosY(),360) || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360))
                  robot.UpdateHardRobotLocation();
                  apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd); 
                  [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
                  if (retCode!=0)
                    [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
                    if(issue)
                 %     emergencyStop=true;
                      automatonState=apGet(apRobot,"automatonState");
                      apRobot = setfield(apRobot,"automatonState",[lost,automatonState(2),automatonState(3)]);  
                      break;
                    endif
                    robot.UpdateHardRobotLocation();
                    [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
                  endif
                end
                particles=ResampleParticles(apGet(apRobot,"img"),plotValue>=2,apGet(apRobot,"particles"));
              
                if (mod(360+newH,360)-mod(360+headingNO,360)>15)  % to much difference between robot calculation and NO
                 printf(mfilename);
                  printf(" inconsitancy heading:%d headingNO:%d NO:%d SpaceNO:%d *** ",newH,headingNO,robot.northOrientation,spaceNO)
                  printf(ctime(time()))
  %								headingIssue=true
                endif
  %							robot.SetAlpha(robot.GetHardHeading())								
    					endif

          else
            printf(mfilename);
            printf(" robot location inconsistency X:%f expected:%f Y:%f expected:%f orientation:%f expected:%f. *** ",robot.GetHardPosX,newX,robot.GetHardPosY,newY,robot.GetHardHeading,newH)
            printf(ctime(time()))
            issue=1;
            automatonState=apGet(apRobot,"automatonState");
            apRobot = setfield(apRobot,"automatonState",[locked,automatonState(2),automatonState(3)]);  
          endif
      printf(mfilename);   
      printf(" total cpu:%f . *** ",cputime()-cpu1);
      printf(ctime(time()))
  end
  robot.Horn(20);  % horn 20 seconds
 % AStarShowStep(trajectory,"determined trajectory");
  ApAStarShowStep(apRobot,trajectory,"determined trajectory") 
  save ("-mat4-binary","traceDet.mat","traceDet");
  save ("-mat4-binary","traceMove.mat","traceMove");
  save ("-mat4-binary","traceNext.mat","traceNext");
  save ("-mat4-binary","traceRobot.mat","traceRobot");
  save ("-mat4-binary","traceEcho.mat","traceEcho");
  hold off
  return;
 endfunction