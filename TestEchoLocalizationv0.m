 function [] = TestEchoLocalization(simulationMode)
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
[robot,carto,headingNOrthXOrientation,zonesXY,all_theta] = InitOctaveRobot();
robot.LaunchBatch();        % call java method to start batch
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
newX=150;
newY=150;
newH=0;
newProb=00;
locProb=newProb;
probExpectedMoveOk=50;
probHardMoveOk=50;
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
particles = CreateLocatedParticles(carto,-1,-1,0,90,particlesNumber,plotOn); % create particles everywhere north oriented
if (retCode==0)
	aligned=true;
	else
	[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom)
	if action=="stop.."
		return
	endif
endif

while (locProb<95)
	retCode=9;
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
		printf("robot localize orientation:%d . ",orientation)
		printf(ctime(time()))
		firstStep=false;
		retry=0;
		while (firstStep==false && issue==false)
			[echoX,echoY,echoH,echoProb,retCode]=EchoLocalizeRobotWithRotation(robot,nbPred,orientation);
			for i=1:nbPred
				printf("echo localize X:%d Y:%d heading:%d prob:%d weight:%d. ",echoX(i),echoY(i),echoH(i),echoProb(i));
				printf(ctime(time()))			
			endfor
			[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
			printf("step 2 particles det location is X:%d Y:%d heading:%d  ",detX,detY,detH)
			printf(ctime(time()))
				weight = TestLocationEchoConsistancy(robot,carto,detX,detY,detH);
				printf("det localize X:%d Y:%d heading:%d weight:%d. ",detX,detY,detH,weight);
				printf(ctime(time()))			
			particles=ResampleParticles(plotOn,particles);
%			analyseRetcode(robot,retCode,WaitScan360)
			if (retCode!=0)
				issue=true;
			endif
			deltaNO=mod(360+SpaceNorthOrientation(zonesXY,detX(1),detY(1))-detH(1),360)   % compute the difference between current orientation and orientation during the learning phase
%			deltaNO=mod(NO+360-SpaceNorthOrientation(echoX(1),echoY(1)),360);
%			for i=1:nbPred
%				weight(i) = TestLocationEchoConsistancy(robot,carto,echoX(i),echoY(i),echoH(i));
%			endif
%			if (prob>=85)
%				echoH(1)=prob;
%				firstStep=true;			
%			endif
			prev=[detX(1),detY(1)]				% keep coordinates before next echoloc
			if (abs(deltaNO)<=3 || abs(deltaNO)>= 358)     % if closed to
				if (retry==0)           					%  best prediction is good enough
					firstStep=true;
				else										% orientation as already been adjusted according to the learning pahse
					if ([echoX(1),echoY(1)]==prev) 			% after adjustement the best prediction is good enough
						firstStep=true;
					endif
				endif
			else
				retry++;
				robot.NorthAlign(SpaceNorthOrientation(zonesXY,prev(1),prev(2)));  % align according to the trainning for this location
				WaitFor=WaitNorthAlign;
				retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
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
				NO=robot.GetNorthOrientation;          			% request info
				WaitFor=WaitInfo;
				retCode=WaitForRobot(robot,WaitFor); 			% wait for up to date
				deltaRotation=robot.GetDeltaNORotation()
				particles=MoveParticles(deltaRotation,0,plotOn,particles);
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
					if (action=="noMove." )
						printf (action);
					endif
					if (action=="inMove" || action=="osbta")
						printf (action);
					endif
				endif
				NO=robot.GetNorthOrientation;           		% get the up to date info
			endif
			if (retry>=3)
				firstStep=true;
				retry=-1;
			endif
		end
		if (retCode!=0 || retry==-1)	
			issue=true;
			return
		else
			robot.SetPosX(echoX(1));
			robot.SetPosY(echoY(1));
			robot.SetHeading(echoH(1));
			robot.SetCurrentLocProb(echoProb(1));
			printf("update hard robot.. ")
			printf(ctime(time()))
			robot.UpdateHardRobotLocation();
			WaitFor=WaitInfo;
			WaitForRobot(robot,WaitFor);
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
		endif
		for i=1:nbPred
			printf("robot echo location is X:%d Y:%d orientation:%d with %d%% probability. ",echoX(i),echoY(i),echoH(i),echoProb(i))
			printf(ctime(time()))
		endfor
		printf("find robot location. \n")
		[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(newX,newY,newProb,plotOn,particles);
		printf("step 2 particles det location is X:%d Y:%d heading:%d  ",detX,detY,detH)
		printf(ctime(time()))
		particles=ResampleParticles(plotOn,particles);
		locProb=newProb

		if (newProb>=85 && issue==false)
			printf("determined location is X:%d Y:%d orientation:%d with %d%% probability. ",detX,detY,detH,newProb)
			printf(ctime(time()))
			robot.Horn(20);  % horn 20 seconds
			break
			else
				lenToDo=11;
				rotation=0;
				printf("move rotation:%d distance:%d. ",rotation,lenToDo)
				printf(ctime(time()))
%				retCode=9;
				robot.Move(rotation,lenToDo);  % len sent in cm
				particles=MoveParticles(rotation,lenToDo,plotOff,particles);
				WaitFor=WaitMove;
				retCode=WaitForRobot(robot,WaitFor)
%				rotation=mod(rotation+90,360)
				if (retCode!=0)
					[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom)
					if action=="stop.."
						return
					endif
					if action=="break."
						break
					endif
					if action=="resume"
						pause
					endif
				endif
		endif	
	end	
endfunction