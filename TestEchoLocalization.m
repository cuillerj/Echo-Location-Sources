 function [] = TestEchoLocalization(simulationMode)
% stepLen=20;
%{
main steps
	create particles everywhere oriented as during the learning phase 
	align robot as during the learning phase based on compas
	scan 360 and use logistic regresion to guess weighted possible locations 
	take into account the compas orientation recorded during the learning phase to update the possible locations weights
	determine possibles locations particles taking into account updated weights


%}

flat=true    % determine regression logistic will be based on flat or rotated logic
 cpu1=cputime();
   if (exist("simulationMode"))     
  else
    simulationMode=0;
  endif
 %simulationMode=1;  % to set simulation 1 on 0 off
 if (simulationMode==1)
 	printf("Simulation mode *** ")
 	printf(ctime(time()))
endif
particlesNumber=2000;
plotOn=true;                 % if true graphics will be provided (reduces performance)
plotOff=false;               % no graphic
[robot,carto,img,shiftNorthXOrientation,zonesXY,all_theta,parametersNameList,parametersValueList] = InitOctaveRobot(flat);
[p1,shiftEchoVsRotationCenter,p3]=GetParametersValueByName(robot,"shiftEchoVsRotationCenter",parametersNameList);
shiftEchoVsRotationCenter=shiftEchoVsRotationCenter/10;  %  distance beetwen robot rotation center and encho center
[p1,avgNorthOrientation,p3]=GetParametersValueByName(robot,"avgNorthOrientation",parametersNameList); % average of all NO during the carto scan (select avg(orientation) where idscan >0) => 0 heading in X-Y
[p1,minRotToBeDone,p3]=GetParametersValueByName(robot,"minRotToBeDone",parametersNameList); % minimum rotation allowed
robot.LaunchBatch();        % call java method to start batch
particles = CreateLocatedParticles(carto,img,-1,-1,0,100,particlesNumber,plotOn); % create particles everywhere oriented as average during learning scan phase
if (simulationMode==0)                  % in case real mode
	robotStatus=0;
	while (robotStatus<=0)              % wait for robot to be ready
		robotStatus=robot.runningStatus;
		pause(1);
	end
robot.QueryMotorsPWM();      % to updload current PMW settings
endif
[rc] = InitRobotParameters(robot,parametersNameList); % set robot parameters
nbLocPossibility=size(all_theta,1); % determine the number of zones used during the trainning phase
%predLocMatrix=InitLocMatrix(all_theta);
%j=1;
               
%targetReached=false;        
%ready=false;
nbPred=10;                    % define the number of predictions that will be compute for each 360° scan

printf(ctime(time()))

shiftNorthXOrientation
headingNOrthXOrientation=shiftNorthXOrientation
WaitInfo=robot.robotInfoUpdated;
WaitRefreshNO=robot.robotNOUpdated;               
WaitScan360=robot.scanEnd;
WaitMove=robot.moveEnd;
WaitNorthAlign=robot.northAlignEnd;
WaitServoAlign=robot.servoAlignEnd;
WaitRobotUpdate=robot.robotUpdatedEnd;
WaitFor=0;
callFrom=10;          % to identify the main octave function
%cartoId=1;
%headingIssue=false;
%headingNO=0; 		 % heading regarding magneto mesurment and value for echo scan reference
%spaceNO=0;
robot.SetSimulationMode(simulationMode);  % set java code mode 0 normal 1 simulation mode
%[posX,posY,heading,locProb] = InitCurrentLocation(carto,robot)  % manualy init position if prob >0
newX=184;
newY=184;
newH=0;
newProb=00;
%locProb=newProb;
probExpectedMoveOk=50;
%probHardMoveOk=50;
traceDet=[];
traceMove=[];
traceEcho=[];
printf("robot location is X:%d Y:%d orientation: %d. prob:%d *** ",newX,newY,newH,newProb);
printf(ctime(time()));
printf("align robot:%d  ",headingNOrthXOrientation)
printf(ctime(time()))
robot.NorthAlign(headingNOrthXOrientation);    % align robot such that it will be in the same orientation (average valu) as for learning phase
count=0;
aligned=false;
issue=false;             % flag used to identify any issues
WaitFor=WaitNorthAlign;
retCode=WaitForRobot(robot,WaitFor);
determinedPosList=[];
expectedPosList=[];
delta=[];
if (retCode==0)
	aligned=true;
	else
	[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom)
	if action=="stop.."
		return
	endif
endif
count=0;
%robot.SetObstacleDetection(0);  % disable obstacle detection
located=false;
while (count<5 && located==false)
%	retCode=9;
	save ("-mat4-binary","traceDet.mat","traceDet");
	save ("-mat4-binary","traceMove.mat","traceMove");
	save ("-mat4-binary","traceEcho.mat","traceEcho");

	count++;

%		firstStep=false;
			[echoX,echoY,echoH,echoProb,retCode]=EchoLocalizeRobotWithRotation(robot,nbPred,0);
			NOCoef=zeros(1,nbPred);
			for i=1:nbPred
				printf("echo localize X:%d Y:%d heading:%d prob:%d weight:%d. ",echoX(i),echoY(i),echoH(i),echoProb(i));
				printf(ctime(time()))
				NOValue = SpaceNorthOrientation(zonesXY,echoX(i),echoY(i));
				NOCoef(i)=max((1-(NOValue-headingNOrthXOrientation)^2/headingNOrthXOrientation),0.01);
			endfor
			traceEcho=[traceEcho;[time,echoX,echoY,echoProb,NOCoef]];
			echoProb=echoProb.*NOCoef
			[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,img,plotOn,particles);
			robot.SetPosX(round(detX));
			robot.SetPosY(round(detY));
			robot.SetHeading(round(detH));
			if (detX==[] || isnan(detX))
				printf("Not found . ");
				printf(ctime(time()));
				count=99;
				return
			endif
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
			traceDet=[traceDet;[time,count,detX,detY,detH]];
			printf("step:%d particles det location is X:%d Y:%d heading:%d  ",count,detX,detY,detH)
			printf(ctime(time()))
			detHRadian=detH*pi()/180;
			determinedPosList=[determinedPosList;[detX,detY,detHRadian]];
			if (count>1)
				delta=[delta;[sqrt((detX-expectedPosList(count-1,1))^2+(detY-expectedPosList(count-1,2))^2)]]
			endif
			particles=ResampleParticles(img,plotOff,particles);
			[rotation,lenToDo] = DetermineNextMoveToScan(robot,plotOn)
			expectedPosList=[expectedPosList;[detX+lenToDo*cos(detHRadian+rotation),detY+lenToDo*sin(detHRadian+rotation)]];
			[rotation,lenToDo] = OptimizeMove(rotation,lenToDo)
			rotation=rotation*180/pi();
			if (abs(rotation)<minRotToBeDone)
				rotation=0
			endif
			printf("move rotation:%d distance:%d. ",rotation,lenToDo)
			printf(ctime(time()))
%				retCode=9;
			if (rotation!=0)
				printf("gyro rotate robot  rotation:%d  *** ",rotation)
				printf(ctime(time()));
				robot.Horn(1);  % horn 1 seconds
				pause(2)			
				robot.RobotGyroRotate(round(rotation));    % rotation based gyroscope
				WaitFor=WaitMove;
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
			robot.Horn(1);  % horn 1 seconds
			pause(2);
			robot.Move(0,lenToDo);  % len sent in cm
			forward=sign(lenToDo);
			lastParticles=particles;          % to be able to recover in case of move failure			
			particles=MoveParticles(round(rotation),round(lenToDo),img,plotOff,particles,shiftEchoVsRotationCenter);
			WaitFor=WaitMove;
			retCode=WaitForRobot(robot,WaitFor);
%				rotation=mod(rotation+90,360)
			if (retCode>=99)  
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
			newLenToDo=0;
			if (retCode==robot.moveKoDueToSpeedInconsistancy )
				newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
				printf("incompleted DueToSpeedInconsistancy expected: %d actual:%d %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
				printf(ctime(time()))
				particles=MoveParticles(rotation,newLenToDo,img,plotOff,lastParticles,shiftEchoVsRotationCenter);
				probExpectedMoveOk=25;
				gyroLenToDo=newLenToDo;
			endif	
			if (retCode==robot.moveKoDueToObstacle )
				newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
				printf("incompleted move due to obstacle expected: %d actual:%d  %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
				printf(ctime(time()))
				particles=MoveParticles(rotation,newLenToDo,img,plotOff,lastParticles,shiftEchoVsRotationCenter);
				probExpectedMoveOk=25;				
				gyroLenToDo=newLenToDo;
			endif	
			traceMove=[traceMove;[time,count,rotation,lenToDo,newLenToDo]];
			NorthOrientation = SpaceNorthOrientation(zonesXY,detX(1),detY(1))
			NO=robot.RefreshNorthOrientation;          %  request info
			WaitFor=WaitRefreshNO;
			retCode=WaitForRobot(robot,WaitFor); 	% wait for up to date
			if (retCode!=0)
				[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom)
				if action=="stop"
					return
				endif
				if action=="break"
					break
				endif
				if action=="resume"
					resume
				endif
			endif
			prevNO=robot.northOrientation;          %  request info
%			robot.NorthAlign(NorthOrientation);    % align robot such that it will be in the same orientation (average valu) as for learning phase
			robot.NorthAlign(headingNOrthXOrientation);  
			WaitFor=WaitNorthAlign;
			retCode=WaitForRobot(robot,WaitFor);
			if (retCode==0)
				aligned=true;
			else
				[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom)
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
			deltaNOAfterNOAlign=prevNO-robot.northOrientation
			particles=MoveParticles(-floor(rotation),0,img,plotOff,particles,shiftEchoVsRotationCenter);
			if (count>2 && delta(count-1)<=18 && delta(count-1)<=delta(count-2) && delta(count-2)<=delta(count-3))
				located=true;
			endif
end	
determinedPosList
det=determinedPosList(:,1:2)
expectedPosList
delta
AStarShowStepV2("Determined & expected postiions ",det,"r",expectedPosList,"g")
robot.Horn(20);  % horn 20 seconds
endfunction