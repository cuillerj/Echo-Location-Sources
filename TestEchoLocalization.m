 function [] = TestEchoLocalization(simulationMode)
% stepLen=20;
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
% headingNOrthXOrientation=267; % average heading NO 
[robot,carto,img,shiftNorthXOrientation,zonesXY,all_theta,parametersNameList,parametersValueList] = InitOctaveRobot();
robot.LaunchBatch();        % call java method to start batch
if (simulationMode==0)                  % in case real mode
	robotStatus=0;
	while (robotStatus<=0)              % wait for robot to be ready
		robotStatus=robot.runningStatus;
		sleep(1);
	end
[rc] = InitRobotParameters(robot,parametersNameList); % set robot parameters
nbLocPossibility=size(all_theta,1); % determine the number of zones used during the trainning phase
predLocMatrix=InitLocMatrix(all_theta);
j=1;
issue=0;                    % flag used to identify any issues
targetReached=false;        
ready=false;
nbPred=10;                    % define the number of predictions that will be compute for each 360° scan
plotOn=true;                 % if true graphics will be provided (reduces performance)
plotOff=false;               % non graphic
printf(ctime(time()))
particlesNumber=1000;
shiftNorthXOrientation
headingNOrthXOrientation=267-shiftNorthXOrientation
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
newX=150;
newY=150;
newH=0;
newProb=00;
locProb=newProb;
probExpectedMoveOk=50;
probHardMoveOk=50;
particles = CreateLocatedParticles(carto,img,-1,-1,0,80,particlesNumber,plotOn); % create particles everywhere north oriented
printf("robot location is X:%d Y:%d orientation: %d. prob:%d *** ",newX,newY,newH,newProb);
printf(ctime(time()));
printf("align robot:%d  ",headingNOrthXOrientation)
printf(ctime(time()))
robot.NorthAlign(headingNOrthXOrientation);    % align robot such that it will be in the same orientation (average valu) as for learning phase
count=0;
aligned=false;
issue=false;
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
while (count<10 && located==false)
%	retCode=9;
	count++;
	NO=robot.GetNorthOrientation;          %  request info
	WaitFor=WaitInfo;
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
		NO=robot.GetNorthOrientation;           % get the up to date info
		orientation=mod(NO+360-headingNOrthXOrientation,360);
		printf("robot orientation:%d . ",orientation)
		printf(ctime(time()))
		firstStep=false;
			[echoX,echoY,echoH,echoProb,retCode]=EchoLocalizeRobotWithRotation(robot,nbPred,orientation);
			for i=1:nbPred
				printf("echo localize X:%d Y:%d heading:%d prob:%d weight:%d. ",echoX(i),echoY(i),echoH(i),echoProb(i));
				printf(ctime(time()))			
			endfor
			[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,img,plotOn,particles);
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
			printf("move rotation:%d distance:%d. ",rotation,lenToDo)
			printf(ctime(time()))
%				retCode=9;

			robot.Move(rotation,lenToDo);  % len sent in cm

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
			particles=MoveParticles(floor(rotation),floor(lenToDo),img,plotOff,particles);
			NorthOrientation = SpaceNorthOrientation(zonesXY,detX(1),detY(1))
			prevNO=robot.GetNorthOrientation;          %  request info
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
			deltaNOAfterNOAlign=prevNO-robot.GetNorthOrientation
			particles=MoveParticles(-floor(rotation),0,img,plotOff,particles);
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