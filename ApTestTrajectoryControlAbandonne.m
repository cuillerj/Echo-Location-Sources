   function [apRobot,robot] = ApTestTrajectoryControl(simulationMode)
   more off
   debugOn=false;
   cpu1=cputime();
   emergencyStop=false;
    if (exist("simulationMode"))
    else
      simulationMode=1;
    endif
    realMode=!simulationMode;
   %simulationMode=1;  % to set simulation 1 on 0 off
   rotationType=2  % select 1="northOrientation" 2="gyroscope" 3="wheels"
   flat=true    % determine regression logistic flat or not 
  [apRobot,robot] = ApInitApRobot(flat,realMode);
  if (realMode)
      robot.LaunchBatch();        % call java method to start batch
    else
      robot.LaunchSimu();
  endif
  if (simulationMode==0)                  % in case real mode
    robotStatus=0;
    while (robotStatus<=0)              % wait for robot to be ready
      robotStatus=robot.runningStatus;
      pause(1);
    end
  robot.QueryMotorsPWM();
  else
    robot.noiseLevel=0.0;                 % coefficent (float) of noise of simualtion mode 0 no noise
    robot.noiseRetCode=0;                 % boolean (0 move 100% completed 1 move can be incompleted)
    robot.noiseRetValue=12;               % range of noised retcode in wich a random retcode is choosen
    printf("Simulation noiseLevel:%f noiseRetCode:%d noiseRetValue:%d *** ",robot.noiseLevel,robot.noiseRetCode,robot.noiseRetValue);
    printf(ctime(time()));
  endif
  all_theta=apGet(apRobot,"all_theta");
  nbLocPossibility=size(all_theta,1); % determine the number of zones used during the trainning phase
  predLocMatrix=InitLocMatrix(all_theta);
  j=1;
  issue=0;                    % flag used to identify any issues
  targetReached=false;        
  ready=false;
  nbPred=5;                    % define the number of predictions that will be compute for each 360° scan
  plotOn=true;                 % if true graphics will be provided (reduces performance)
  plotOff=false;               % non graphic
  printf(ctime(time()))
  particlesNumber=1000;
  WaitInfo=robot.robotInfoUpdated;                 
  WaitScan360=robot.scanEnd;
  WaitMove=robot.moveEnd;
  WaitNorthAlign=robot.northAlignEnd;
  WaitServoAlign=robot.servoAlignEnd;
  WaitRobotUpdate=robot.robotUpdatedEnd;
  WaitFor=0;
  callFrom=10;          % to identify the main octave function
  %cartoId=1;
  headingIssue=false;
  headingNO=0; 		 % heading regarding magneto mesurment and value for echo scan reference
  spaceNO=0;
  robot.SetSimulationMode(simulationMode);  % 0 normal 1 siimulation mode
  %[posX,posY,heading,locProb] = InitCurrentLocation(carto,robot)  % manualy init position if prob >0
  newX=50;
  newY=50;
  newH=0;
  newProb=100;
  apRobot = setfield(apRobot,"location",[newX,newY,newH]);                        % defined / determined location 
  apRobot = setfield(apRobot,"locationProb",newProb);    
  probExpectedMoveOk=50;
  probHardMoveOk=50;
  shiftNO=0;
  printf("robot location is X:%d Y:%d orientation: %d. prob:%d *** ",newX,newY,newH,newProb);
  printf(ctime(time()));
 % particles = CreateLocatedParticles(carto,img,newX,newY,newH,newProb,particlesNumber,plotOff);
  [apRobot] = ApCreateLocatedParticles(apRobot,particlesNumber,plotOn);
  %particles = CreateLocatedParticles(carto,img,-1,-1,-1,0,particlesNumber,plotOn);
  zonesXY=apGet(apRobot,"scanRefPoints");
  spaceNO=SpaceNorthOrientation(zonesXY,newX,newY);  % 
  trajectory=[newX,newY];
  traceDet=[];
  traceMove=[];
  traceNext=[];
  traceRobot=[];
  traceEcho=[];
  if (simulationMode==0)
    echoBalance=[1,0.95,1.05,1.1,1.1];  % theoritical , encoder , gyroscope theo , BNO Left, BNO Right
  else
    echoBalance=[1,0.95,1.05];  % theoritical , encoder , gyroscope
  endif
  
  gyroBasedX=newX;
  gyroBasedY=newY;
  gyroBasedH=newH*pi()/180;
  if (simulationMode==0)                  % in case real mode
    robotStatus=0;
    while (robotStatus<=0)              % wait for robot to be ready
      robotStatus=robot.runningStatus;
      pause(1);
    end
  [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);
  endif
  if (simulationMode!=0)
    NO=normrnd(spaceNO,5);  % simulate noise NO
    headingNO=mod(360+spaceNO-NO,360);   % determine heading regarding magneto mesurment and value for echo scan reference
    shiftNO=spaceNO-NO
  else
    NO=robot.northOrientation;          %  request NO info
 %   WaitFor=WaitInfo;
   apRobot = setfield(apRobot,"waitFor",WaitInfo); 
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
  posX=newX;
  posY=newY;
  locProb=newProb;
  heading=newH;
  retCode=99;
  %robot.northOrientation=NO;  
  
  printf("robot shiftNO:%d *** ",spaceNO);
  printf(ctime(time()));
  %{
      compute target location
  %}
  %[targetX,targetY,targetAngle]=ComputeTargetLocation(carto,robot);
  targetX=585;
  targetY=135;
 % targetX=100;
 % targetY=55;
  targetH=00;;
  apRobot = setfield(apRobot,"destination",[targetX,targetY,targetH]); 
  apRobot = setfield(apRobot,"newTarget",true);  
  printf("robot target is X:%d Y:%d orientation: %d. *** ",targetX,targetY,targetH)
  printf(ctime(time()))
  %
  [apRobot,robot,retCode] = ApComputeOptimalPath(apRobot,robot,[targetX,targetY,targetH],plotOn);
  robot.SetPosX(posX);
  robot.SetPosY(posY);
  robot.SetHeading(heading);
  robot.SetCurrentLocProb(locProb);
  printf("update hard robot. *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
  printf(ctime(time()))
  while((robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360)) && !emergencyStop)
    robot.UpdateHardRobotLocation();	
    apRobot = setfield(apRobot,"waitFor",WaitRobotUpdate);
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
          printf("update hard robot.. *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading());
          printf(ctime(time()));
          while((robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360)) && !emergencyStop)
            robot.UpdateHardRobotLocation();
            [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
            if (retCode!=0)
              [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
               if(issue)
                  emergencyStop=true;
                  break
               endif
            endif
          end
          printf("hardPosX:%d hardPosY:%d *** ",robot.GetHardPosX,robot.GetHardPosY);
          printf(ctime(time()))
  endif
  loopCount=0;	
  traceNext=[traceNext;[time,loopCount,posX,posY]];
  gyroBasedX=gyroBasedX-apGet(apRobot,"shiftEchoVsRotationCenter")*cos(newH);           % set position to rotation center
  gyroBasedY=gyroBasedY-apGet(apRobot,"shiftEchoVsRotationCenter")*sin(newH);           % set position to rotation center
  while (targetReached==false && !emergencyStop)
    loopCount++;
    rotation=0;
        if ((abs(robot.GetHardPosX-round(newX))<=1  && abs(robot.GetHardPosY-round(newY))<=1 ))
              if (ApQueryCartoAvailability(apRobot,[newX,newY,newH],false,1))
                [available,retCode] = ApQueryCartoAvailability(apRobot,[newX,newY,newH],false,true);
                currentPositionIssue=true;
              endif
              if ((targetX-robot.GetHardPosX)^2 + (targetY-robot.GetHardPosY)^2 <=225)
                targetReached=true;
                % check target reached
              else 
                % compute trajectory step
                if (simulationMode==0)
                  robot.Horn(3);  % horn 20 seconds
                  pause(3);
  %							inp=input("take a picture ");
                endif
                [apRobot,robot,next,direct,forward] = ApComputeNextStepToTarget(apRobot,robot,plotOn);
                newTarget=0;
                if (forward==-9)
                  printf("no path found. *** ")
                  printf(ctime(time()))
                  issue=true;
  %								pause
                  AStarShowStep(trajectory,"determined trajectory");
                  save ("-mat4-binary","traceDet.mat","traceDet");
                  save ("-mat4-binary","traceMove.mat","traceMove");
                  save ("-mat4-binary","traceNext.mat","traceNext");
                  save ("-mat4-binary","traceRobot.mat","traceRobot");
                  save ("-mat4-binary","traceEcho.mat","traceEcho");
                  return
                endif
                printf("Next step is n°:%d X:%d Y:%d . *** ",loopCount,next(1),next(2))
                printf(ctime(time()))
                traceNext=[traceNext;[time,loopCount,next]];
  %							if (direct==false)
                [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward);
                saveLocation=[robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading];
                traceMove=[traceMove;[time,loopCount,rotationToDo,lenToDo]];
                printf("rotation:%d distance:%d *** ",rotationToDo,lenToDo);
                printf(ctime(time()));
                saveTargetHeading=robot.GetHeading()+rotationToDo;  % replace rotation by rotationToDo le 28/09
                saveHeading=robot.GetHeading();
%                 if (simulationMode==1)
    %							rotationToDo=rotationParticlesToDo;
  %              endif
                if (rotationToDo!=0)
				           robot.Horn(1);  % horn x seconds
                   [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOn);
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
                     printf("stop due to pb rotation")
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
                  [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOn);
                endif
                while (retCode==robot.moveKoDueToNotEnoughSpace)
                  if (robot.retCodeDetail>=apGet(apRobot,"minDistToBeDone"))
                    {
                    lenToDo=robot.retCodeDetail;
                    printf(" try new length  x:%d *** ",lenToDo)
                    printf(ctime(time()))
                    [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOn);
                    gyroLenToDo=lenToDo; 
                    next(1)=round(apGet(apRobot,"saveLocation")(1)+cos((apGet(apRobot,"saveLocation")(3)+rotationToDo)*pi()/180)*lenToDo)
                    next(2)=round(apGet(apRobot,"saveLocation")(2)+sin((apGet(apRobot,"saveLocation")(3)+rotationToDo)*pi()/180)*lenToDo)
                    }           
                  else{
                    issue=true;
                    printf(" no possible move max distance:%d *** ",robot.retCodeDetail);
                    printf(ctime(time()));
                    retCode=99;
                   }
                  endif       
                end
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
                  printf("Subsystem location leftX:%d leftY:%d rightX:%d rightY:%d heading:%d",subsystemLeftX,subsystemLeftY,subsystemRightX,subsystemRightY,subsystemHeading);
                  printf(" *** ");
                  printf(ctime(time()));
                  gyroBasedH=mod(gyroBasedH+robot.GetGyroHeading()*pi()/180,2*pi());
                  gyroBasedX=round(gyroBasedX+gyroLenToDo*cos(gyroBasedH)+apGet(apRobot,"shiftEchoVsRotationCenter")*cos(gyroBasedH));
                  gyroBasedY=round(gyroBasedY+gyroLenToDo*sin(gyroBasedH)+apGet(apRobot,"shiftEchoVsRotationCenter")*sin(gyroBasedH));
                  if (simulationMode==0)
                    newX=[next(1),robot.GetHardPosX(),gyroBasedX,subsystemLeftX,subsystemRightX];
                    newY=[next(2),robot.GetHardPosY(),gyroBasedY,subsystemLeftY,subsystemRightY];
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
                    printf("the hard position is theoriticaly not possible. *** x:%d y:%d h:%d ",robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading())
                    printf(ctime(time()))
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
                  printf ("Heading hard H:%d  NO:%d Theoretical H:%d  Average:%f SpaceNO:%d RobotNO:%d Gyro:%d*** ",newH,headingNO,saveTargetHeading,averageH*180/pi(),spaceNO,robot.northOrientation,robot.GetGyroHeading());
                  printf(ctime(time()))
    %							[weightEcho,retValue,echo] = TestLocationEchoConsistancy(robot,carto,newX,newY,averageH)
                  [apRobot,robot,weightEcho,retValue,echo,quality] = ApTestLocationEchoConsistancyVsDB(apRobot,robot,newX,newY,averageH);
                  if (retValue==0)
                    traceEcho=[traceEcho;[time,loopCount,newX,newY,weightEcho,echo,quality]];
                  else
                      printf("no echo consistancy data *** ");
                      printf(ctime(time()));
                      for i=1:col	
                      weightEcho(i)=1;
                      endfor;	                
                  endif
                [nbRow,nbCol]=size(weightEcho);
                printf("best quality (0: one is perfect):%f",quality);
                printf(" *** ");
                printf(ctime(time()));
                weightEcho=weightEcho.*echoBalance;   % 
                [apRobot,robot,detX,detY,detH] = ApDetermineRobotLocationWithParticlesGaussian(apRobot,robot,newX,newY,weightEcho,plotOn);
                traceDet=[traceDet;[time,loopCount,detX,detY,detH]];
                printf("determined X: %f ",round(detX))
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
                printf("update hard robot... *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
                printf(ctime(time()))
                while(robot.GetPosX()!=robot.GetHardPosX() || mod(robot.GetPosY(),360)!=mod(robot.GetHardPosY(),360) || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360))
                  robot.UpdateHardRobotLocation();	
                  [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
                  if (retCode!=0)
                    [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode);
                    if(issue)
                      emergencyStop=true;
                      break;
                    endif
                    robot.UpdateHardRobotLocation();	
                    [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);
                  endif
                end
                particles=ResampleParticles(apGet(apRobot,"img"),plotOff,apGet(apRobot,"particles"));
              
                if (mod(360+newH,360)-mod(360+headingNO,360)>15)  % to much difference between robot calculation and NO
                  printf("inconsitancy heading:%d headingNO:%d NO:%d SpaceNO:%d *** ",newH,headingNO,robot.northOrientation,spaceNO)
                  printf(ctime(time()))
  %								headingIssue=true
                endif
  %							robot.SetAlpha(robot.GetHardHeading())								
    					endif

          else
            printf("robot location inconsistency X:%f expected:%f Y:%f expected:%f orientation:%f expected:%f. *** ",robot.GetHardPosX,newX,robot.GetHardPosY,newY,robot.GetHardHeading,newH)
            printf(ctime(time()))
            issue=1;
          endif
          
      printf("total cpu:%f . *** ",cputime()-cpu1);
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
