 function [] = TestTrajectoryControl(simulationMode)
   more off
 cpu1=cputime();
   if (exist("simulationMode"))
  else
    simulationMode=1;
  endif
 %simulationMode=1;  % to set simulation 1 on 0 off
 rotationType=2  % select 1="northOrientation" 2="gyroscope" 3="wheels"
 flat=true    % determine regression logique flat or not 
[robot,carto,img,headingNOrthXOrientation,zonesXY,all_theta,parametersNameList,parametersValueList] = InitOctaveRobot(flat);
[p1,shiftEchoVsRotationCenter,p3]=GetParametersValueByName(robot,"shiftEchoVsRotationCenter",parametersNameList);
shiftEchoVsRotationCenter=shiftEchoVsRotationCenter/10;
[param,value,number]=GetParametersValueByName(robot,"IMUMode",parametersNameList);
IMUMode=value;
[param,value,number]=GetParametersValueByName(robot,"NDOFMode",parametersNameList);
NDOFMode=value;
[param,value,number]=GetParametersValueByName(robot,"CompasMode",parametersNameList);
CompasMode=value;
robot.LaunchBatch();        % call java method to start batch
if (simulationMode==0)                  % in case real mode
	robotStatus=0;
	while (robotStatus<=0)              % wait for robot to be ready
		robotStatus=robot.runningStatus;
		pause(1);
	end
robot.QueryMotorsPWM();
else
  robot.noiseLevel=1.0;                 % coefficent (float) of noise of simualtion mode 0 no noise
  robot.noiseRetCode=0;                 % boolean (0 move 100% completed 1 move can be incompleted)
endif

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
newX=55;
newY=55;
newH=0;
newProb=100;
probExpectedMoveOk=50;
probHardMoveOk=50;
shiftNO=0;
printf("robot location is X:%d Y:%d orientation: %d. prob:%d *** ",newX,newY,newH,newProb);
printf(ctime(time()));
particles = CreateLocatedParticles(carto,img,newX,newY,newH,newProb,particlesNumber,plotOff);
%particles = CreateLocatedParticles(carto,img,-1,-1,-1,0,particlesNumber,plotOn);
spaceNO=SpaceNorthOrientation(zonesXY,newX,newY);  % 
trajectory=[newX,newY];
traceDet=[];
traceMove=[];
traceNext=[];
traceRobot=[];
traceEcho=[];
if (simulationMode==0)
	echoBalance=[1,0.95,1.05,1,1]  % theoritical , encoder , gyroscope
else
	echoBalance=[1,0.95,1.05]  % theoritical , encoder , gyroscope
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
[rc] = InitRobotParameters(robot,parametersNameList); % set robot parameters
endif
if (simulationMode!=0)
	NO=normrnd(spaceNO,5);  % simulate noise NO
	headingNO=mod(360+spaceNO-NO,360);   % determine heading regarding magneto mesurment and value for echo scan reference
	shiftNO=spaceNO-NO
else
	NO=robot.GetNorthOrientation;          %  request NO info
	WaitFor=WaitInfo;
	retCode=WaitForRobot(robot,WaitFor); 	% wait for up to date
	if (retCode!=0)
		[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
		if action=="stop.."
			return
		endif
		if action=="break."
			break
		endif
		if action=="resume"
			resume
		endif
	NO=robot.GetNorthOrientation;           % get the up to date info
	headingNO=mod(360+spaceNO-NO,360); 
	shiftNO=spaceNO-NO
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
targetH=00;;
newTarget=1;
printf("robot target is X:%d Y:%d orientation: %d. *** ",targetX,targetY,targetH)
printf(ctime(time()))
%

robot.SetPosX(posX);
robot.SetPosY(posY);
robot.SetHeading(heading);
robot.SetCurrentLocProb(locProb);
printf("update hard robot. *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
printf(ctime(time()))
while(robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || robot.GetHeading()!=robot.GetHardHeading())
	robot.UpdateHardRobotLocation();	
	WaitFor=WaitRobotUpdate;
	retCode=WaitForRobot(robot,WaitFor);
	if (retCode!=0)
		[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
		if action=="stop.."
			return
		endif
		if action=="break."
			break
		endif
		if action=="resume"
			resume
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
				while(robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || robot.GetHeading()!=robot.GetHardHeading())
					robot.UpdateHardRobotLocation();
					WaitFor=WaitRobotUpdate;
					retCode=WaitForRobot(robot,WaitFor);
					if (retCode!=0)
						[issue,action]=analyseRetcode(retCode,WaitFor,callFrom);
						if action=="stop.."
							return
						endif
						if action=="break."
							break
						endif
						if action=="resume"
							resume
						endif
					endif
				end
				printf("hardPosX:%d hardPosY:%d *** ",robot.GetHardPosX,robot.GetHardPosY);
				printf(ctime(time()))
endif
loopCount=0;	
traceNext=[traceNext;[time,loopCount,posX,posY]];
gyroBasedX=gyroBasedX-shiftEchoVsRotationCenter*cos(newH);           % set position to rotation center
gyroBasedY=gyroBasedY-shiftEchoVsRotationCenter*sin(newH);           % set position to rotation center
while (issue==false && targetReached==false)
	loopCount++;
	rotation=0;

			if ((abs(robot.GetHardPosX-round(newX))<=1  && abs(robot.GetHardPosY-round(newY))<=1 ))
						if (QueryCartoAvailability(carto,newX,newY,newH*pi()/180,true)==false)
							currentPositionIssue=true;
						endif
						if ((targetX-robot.GetHardPosX)^2 + (targetY-robot.GetHardPosY)^2 <=225)
							targetReached=true;
							% check target reached
						else 
							% compute trajectory step
							if (simulationMode==0)
								robot.Horn(5);  % horn 20 seconds
								pause(5);
%							inp=input("take a picture ");
							endif
							[nextX,nextY,direct,forward] = ComputeNextStepToTarget(carto,robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading,targetX,targetY,newTarget,plotOn,robot,parametersNameList);
							newTarget=0;
							if (forward==0)
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
							printf("Next step is n°:%d X:%d Y:%d . *** ",loopCount,nextX,nextY)
							printf(ctime(time()))
							traceNext=[traceNext;[time,loopCount,nextX,nextY]];
%							if (direct==false)
							[rotationToDo,lenToDo]=ComputeMoveToDo(robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading,nextX,nextY,forward,robot,parametersNameList);
%							[rotationParticlesToDo,lenParticlesToDo]=ComputeMoveParticlesToDo(robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading,nextX,nextY,forward,robot,parametersNameList)
%							else
%								nextX=targetX;         % straight move possible next step is target
%								nextY=targetY;			% straight move possible next step is target
%							endif
							traceMove=[traceMove;[time,loopCount,rotationToDo,lenToDo]];
							printf("rotation:%d distance:%d *** ",rotationToDo,lenToDo);
							printf(ctime(time()));
							saveTargetHeading=robot.GetHeading()+rotationToDo;  % replace rotation by rotationToDo le 28/09
							saveHeading=robot.GetHeading();
							if (simulationMode==1)
	%							rotationToDo=rotationParticlesToDo;
							endif
							if (rotationToDo!=0)
								printf(ctime(time()))
								if (rotationType==1)
									saveNO=robot.northOrientation;
									printf("NO rotate robot NO: %d rotation:%d  *** ",robot.northOrientation,rotationToDo)
									printf(ctime(time()));									
									robot.RobotNorthRotate(mod(round(rotationToDo)+360,360));    % rotation based on north orientation
									WaitFor=WaitNorthAlign;
								endif
								if (rotationType==2)
									printf("gyro rotate robot  rotation:%d  *** ",rotationToDo)
									printf(ctime(time()));						
									robot.RobotGyroRotate(round(rotationToDo));    % rotation based gyroscope
									WaitFor=WaitMove;
								endif
								if (rotationType==3)
									printf("wheels rotate robot  rotation:%d  *** ",rotationToDo)
									printf(ctime(time()));							
									robot.Move(round(rotationToDo),0);    % rotation based on wheels encoders
									WaitFor=WaitMove;
								endif								
								retCode=WaitForRobot(robot,WaitFor);
								if (retCode==0)
									aligned=true;
									else
									[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
									if action=="stop.."
										return
									endif
								endif
							endif
							if (simulationMode==0)
%								input("ready to move");
							endif
							if (rotationType==1)
								robot.SetHeading(mod(360+robot.GetHardHeading()+robot.northOrientation-saveNO,360));
							endif
							if (rotationType==2)
								robot.SetHeading(mod(360+saveHeading+robot.GetGyroHeading(),360));
								gyroBasedH=gyroBasedH+robot.GetGyroHeading()*pi()/180;
							endif
							if (rotationType==3)
								robot.SetHeading(mod(360+robot.GetHardHeading(),360));
							endif
							traceRobot=[traceRobot;[time,loopCount,1,robot.GetHardPosX(),robot.GetHardPosY(),+robot.GetHardHeading(),robot.GetGyroHeading()],retCode];
							%{
							printf("update hard heading robot. X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
							printf(ctime(time()))
							robot.UpdateHardRobotLocation();	
							WaitFor=WaitRobotUpdate;
							retCode=WaitForRobot(robot,WaitFor);
							if (retCode!=0)
								[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
								if action=="stop.."
									return
								endif
								if action=="break."
									break
								endif
								if action=="resume"
									resume
								endif
							endif
							%}
							gyroLenToDo=lenToDo;
							if (simulationMode==1)
	%							lentoDo=lenParticlesToDo; 		 % len sent in cm
							endif
							robot.Horn(2);  % horn 2 seconds
							pause(5);
							robot.Move(0,lenToDo); 		 % len sent in cm
							lastParticles=particles;          % to be able to recover in case of move failure
		
							particles=MoveParticles(rotationToDo,lenToDo,img,plotOff,particles,shiftEchoVsRotationCenter);
							retCode=WaitForRobot(robot,WaitMove);
							if (retCode>=99)  
								[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
								if action=="stop.."
									return
								endif
								if action=="break."
									break
								endif
								if action=="resume"
									endif
							endif
%							robot.GetGyroHeading
							traceRobot=[traceRobot;[time,loopCount,2,robot.GetHardPosX(),robot.GetHardPosY(),+robot.GetHardHeading(),robot.GetGyroHeading()],retCode];
							probExpectedMoveOk=50;
							if (retCode==robot.moveKoDueToSpeedInconsistancy )
								newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
								printf("incompleted DueToSpeedInconsistancy expected: %d actual:%d %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
								printf(ctime(time()))
								particles=MoveParticles(rotationToDo,newLenToDo,img,plotOff,lastParticles,shiftEchoVsRotationCenter);
								probExpectedMoveOk=25;
								gyroLenToDo=newLenToDo;
							endif	
							if (retCode==robot.moveKoDueToObstacle )
								newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
								printf("incompleted move due to obstacle expected: %d actual:%d  %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
								printf(ctime(time()))
								particles=MoveParticles(rotationToDo,newLenToDo,img,plotOff,lastParticles,shiftEchoVsRotationCenter);
								probExpectedMoveOk=25;				
								gyroLenToDo=newLenToDo;
							endif														
							if (retCode==robot.moveUnderLimitation)
								printf("no move moveUnderLimitation. *** ")
								printf(ctime(time()))
								particles=lastParticles;
								probExpectedMoveOk=1;
							endif
							if (robot.BNOLocFlag==0)
								robot.ValidHardPosition();
							else
								pause(1);
								robot.ValidHardPosition();
							endif
							subsystemLeftX=robot.BNOLeftPosX;
							subsystemLeftY=robot.BNOLeftPosY;
							subsystemRightX=robot.BNORightPosX;
							subsystemRightY=robot.BNORightPosY;
							subsystemHeading=robot.BNOLocHeading;
							printf("Subsystem location leftX:%d leftY:%d rightX:%d rightY:%d heading:%d",subsystemLeftX,subsystemLeftY,subsystemRightX,subsystemRightY,subsystemHeading);
							printf(" *** ");
							printf(ctime(time()));
							gyroBasedH=mod(gyroBasedH+robot.GetGyroHeading()*pi()/180,2*pi());
							gyroBasedX=round(gyroBasedX+gyroLenToDo*cos(gyroBasedH)+shiftEchoVsRotationCenter*cos(gyroBasedH));
							gyroBasedY=round(gyroBasedY+gyroLenToDo*sin(gyroBasedH)+shiftEchoVsRotationCenter*sin(gyroBasedH));
							if (simulationMode==0)
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
							[available,retCode]=QueryCartoAvailability(carto,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading()*pi()/180,1);
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
							[weightEcho,retValue,echo,quality] = TestLocationEchoConsistancyVsDB(robot,carto,newX,newY,averageH,parametersNameList);
							if (retValue==0)
								traceEcho=[traceEcho;[time,loopCount,newX,newY,weightEcho,echo,quality]];
							endif
%{							
							if (median(weightEcho)<50)
								[weightEcho,retValue,echo] = TestLocationEchoConsistancy(robot,carto,newX,newY,averageH);
							endif
							traceEcho=[traceEcho;[time,loopCount,newX,newY,weightEcho,echo]];
							if (median(weightEcho)<50 && max(weightEcho)<80)
								weightEcho=[1,1,1]; 		%non significative mesurments							
							else
								if (((median(weightEcho)-min(weightEcho))/mean(weightEcho)) < 0.3)
									weightEcho=[1,1,1];   %non significative diffrences beetwen mesurments		 
								endif
							endif
%}
							[nbRow,nbCol]=size(weightEcho);
							if (retValue!=0)
								[weightEcho,retValue,echo,quality] = TestLocationEchoConsistancyVsDB(robot,carto,newX,newY,averageH,parametersNameList);
								if (retValue==0)
									traceEcho=[traceEcho;[time,loopCount,newX,newY,weightEcho,echo,quality]];
								else
									printf("no echo consistancy data");
									for i=1:col	
										weightEcho(i)=1;
									endfor;	
								endif
							else
								printf("echo consistancy prob:");
								for i=1:col	
									printf(" %f ",weightEcho(i));
								endfor;
								printf("best quality (0: one is perfect):%f",quality);
							endif
							printf(" *** ");
							printf(ctime(time()));
							weightEcho=weightEcho.*echoBalance   % 
							[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(newX,newY,weightEcho,img,plotOn,particles);
							traceDet=[traceDet;[time,loopCount,detX,detY,detH]];
							printf("detX: %f ",round(detX))
							printf(" detY: %f ",round(detY))
							printf(" detH: %f ",round(detH))
							printf(ctime(time()))
							trajectory=[trajectory;[detX,detY]];
							newX=detX;
							newY=detY;
							newH=detH;
							robot.SetPosX(round(detX));
							robot.SetPosY(round(detY));
							robot.SetHeading(round(detH));
%							printf("det prob: %d *** ",prob);
%							printf(ctime(time()));
%							robot.SetCurrentLocProb(prob(1));
							printf("update hard robot... *** X:%d Y:%d H:%d *** ",robot.GetPosX(),robot.GetPosY(),robot.GetHeading())
							printf(ctime(time()))
							while(robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || robot.GetHeading()!=robot.GetHardHeading())
								robot.UpdateHardRobotLocation();	
								WaitFor=WaitRobotUpdate;
								retCode=WaitForRobot(robot,WaitFor);
								if (retCode!=0)
									[issue,action]=analyseRetcode(retCode,WaitFor,callFrom);
									robot.UpdateHardRobotLocation();	
									WaitFor=WaitRobotUpdate;
									retCode=WaitForRobot(robot,WaitFor);
								endif
							end
							particles=ResampleParticles(img,plotOff,particles);
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
AStarShowStep(trajectory,"determined trajectory");
save ("-mat4-binary","traceDet.mat","traceDet");
save ("-mat4-binary","traceMove.mat","traceMove");
save ("-mat4-binary","traceNext.mat","traceNext");
save ("-mat4-binary","traceRobot.mat","traceRobot");
save ("-mat4-binary","traceEcho.mat","traceEcho");
endfunction
hold off