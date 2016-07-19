cd ..
javaaddpath ('C:\Users\jean\Documents\Donnees\eclipse\RobotServer\bin\robot.jar');
% create and start robot
% load matrix
load carto1;
carto=carto1;
load "zonesXY.txt"
shiftNorthXOrientation=267; % shift between north and X direction in degres - that is the average mesurment
setupPath;					% define paths
robot=robotJava;            % create the java object
robot.SetTraceFileOn(1);    % route console to trace file
robot.LaunchBatch();        % call java method to start batch
load all_theta;             % load the trained logistic regression matrix
nbLocPossibility=size(all_theta,1); % determine the number of zones used during the trainning phase
predLocMatrix=InitLocMatrix(all_theta);
% load "zonesXY.txt"
j=1;
issue=0;                    % flag used to identify any issues
targetReached=false;        
ready=false;
nbPred=5;                    % define the number of predictions that will be compute for each 360° scan
plotOn=true;                 % if true graphics will be provided (reduces performance)
plotOff=false;               % non graphic
printf("create particles. \n")
particlesNumber=1000;
%particles=CreateParticles(carto,particlesNumber,plotOff); % creation for particles filter
%load particles;
WaitInfo=1;                 
WaitScan360=robot.scanEnd;
WaitMove=robot.moveEnd;
WaitNorthAlign=robot.alignEnd;
WaitServoAlign=robot.servoAlignEnd;
WaitUpdate=8;
WaitFor=0;
callFrom=1;          % to identify the main function
%cartoId=1;
headingIssue=false;
shiftNO=0;
while (ready==false)
	ready=yes_or_no(" robot ready to go ?"); % wait for end user to start
	if (ready==false)
		exit
	endif
end
[posX,posY,heading,locProb] = InitCurrentLocation(carto,robot)  % manualy init position if prob >0
newX=posX;
newY=posY;
newH=heading;
newProb=locProb;
particles = CreateLocatedParticles(carto,posX,posY,heading,locProb,particlesNumber,plotOn);
spaceNO=SpaceNorthOrientation(zonesXY,posX,posX);
retCode=9;
NO=robot.GetNorthOrientation;          %  request info
WaitFor=WaitInfo;
retCode=WaitForRobot(robot,WaitFor); 	% wait for up to date
if (retCode!=0)
	[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
shiftNO=NO-spaceNO
%{
		compute target location
%}
[targetX,targetY,targetAngle]=ComputeTargetLocation(carto,robot);
printf("robot target is X:%d Y:%d orientation: %d. \n",targetX,targetY,targetAngle)
%
%{
if (locProb>0 && locProb <100)
		[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(posX,posY,locProb,plotOff,particles);
		printf("step 0 particles det location is X:%d Y:%d heading:%d  \n",detX,detY,detH)
		particles=ResampleParticles(plotOn,particles);
		newX=posX;
		newY=posY;
		newAngle=heading;
		newProb=locProb;
else
		newX=posX;
		newY=posY;
		newAngle=heading;
		newProb=locProb;

endif
%}
robot.SetPosX(posX);
robot.SetPosY(posY);
robot.SetHeading(heading);
robot.SetCurrentLocProb(locProb);
printf("update hard robot. \n")
robot.UpdateHardRobotLocation();	
WaitFor=WaitUpdate;
retCode=WaitForRobot(robot,WaitFor);
if (retCode!=0)
	[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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

% loop till target reached
	% localize robot
	
	
%{
		first step is to north align robot according to the echo learning pahse
%}
if locProb <95
	printf("align robot:%d  \n",shiftNorthXOrientation)
	robot.NorthAlign(shiftNorthXOrientation);    % align robot such that it will be in the same orientation as for learning phase
	count=0;
	aligned=false;
	issue=false;
	WaitFor=WaitNorthAlign;
	retCode=WaitForRobot(robot,WaitFor);
	if (retCode==0)
		aligned=true;
		else
		[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
		if action=="stop.."
			return
		endif
	endif
endif
%{
		robot is aligned start off the driving loop
%}
while (issue==false && targetReached==false)
	%{
	robot location is completly unknown
	localization loop untill probalility is good enough
	%}
	rotation=0;
	while (locProb<95)
		retCode=9;
		NO=robot.GetNorthOrientation;          %  request info
		WaitFor=WaitInfo;
		retCode=WaitForRobot(robot,WaitFor); 	% wait for up to date
		if (retCode!=0)
			[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
		orientation=mod(NO+360-shiftNorthXOrientation,360);
		printf("robot localize orientation:%d . \n",orientation)
		firstStep=false;
		retry=0;
		while (firstStep==false && issue==false)
			[echoX,echoY,echoH,echoProb,retCode]=EchoLocalizeRobotWithRotation(robot,nbPred,orientation);
			for i=1:nbPred
				printf("echo localize X:%d Y:%d heading:%d prob:%d . \n",echoX(i),echoY(i),echoH(i),echoProb(i))
			endfor
%			analyseRetcode(retCode,WaitScan360)
			if (retCode!=0)
				issue=true;
			endif
			deltaNO=mod(360+SpaceNorthOrientation(zonesXY,echoX(1),echoY(1))-NO,360)   % compute the difference between current orientation and orientation during the learning phase
%			deltaNO=mod(NO+360-SpaceNorthOrientation(echoX(1),echoY(1)),360);
			prob = TestLocationEchoConsistancy(robot,carto,echoX(1),echoY(1),echoH(1))
			if (prob>=85)
				echoH(1)=prob;
				firstStep=true;			
			endif
			prev=[echoX(1),echoY(1)]				% keep coordinates before next echoloc
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
					[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
				if (retCode!=0)
					[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
			printf("update hard robot. \n")
			robot.UpdateHardRobotLocation();
			WaitFor=WaitUpdate;
			WaitForRobot(robot,WaitFor);
			if (retCode!=0)
				[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
			printf("robot echo location is X:%d Y:%d orientation:%d with %d%% probability. \n",echoX(i),echoY(i),echoH(i),echoProb(i))
		endfor
		printf("find robot location. \n")
		[newX,newY,newAngle,newProb,predLocMatrix]=DetermineRobotLocation(robot,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardAngle,echoX,echoY,echoH,echoProb,predLocMatrix,shiftNorthXOrientation);
		printf("step 1 particles det location is X:%d Y:%d heading:%d  prob:%d \n",newX,newY,newAngle,newProb)
		[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(newX,newY,newProb,plotOff,particles);
		printf("step 2 particles det location is X:%d Y:%d heading:%d  \n",detX,detY,detH)
		particles=ResampleParticles(plotOn,particles);
		locProb=newProb

		if (newProb>=85 && issue==false)
			printf("determined location is X:%d Y:%d orientation:%d with %d%% probability. \n",newX,newY,newAngle,newProb)
			break
			else
				lenToDo=33;
				printf("move rotation:%d distance:%d. \n",rotation,lenToDo)
%				retCode=9;
				robot.Move(rotation,lenToDo);  % len sent in cm
				particles=MoveParticles(rotation,lenToDo,plotOff,particles);
				WaitFor=WaitMove;
				retCode=WaitForRobot(robot,WaitFor)
				rotation=mod(rotation+90,360)
				if (retCode!=0)
					[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
	%{
	robot location is good enough to start moving toward target
	localization loop as long as probalility is good enough
	%}
			robot.SetPosX(newX);
			robot.SetPosY(newY);
			robot.SetHeading(newH);
			robot.SetCurrentLocProb(newProb);
			printf("update hard robot. \n")
			robot.UpdateHardRobotLocation();
			WaitFor=WaitUpdate;
			retCode=WaitForRobot(robot,WaitFor);
			if (retCode!=0)
				[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
				if (robot.GetHardPosX==newX && robot.GetHardPosY==newY && robot.GetHardAngle==floor(newH))
						if (QueryCartoAvailability(carto,newX,newY,newH*pi()/180,true)==false)
							currentPositionIssue=true;
						endif
						if ((targetX-robot.GetHardPosX)^2 + (targetY-robot.GetHardPosY)^2 <=225)
							targetReached=true;
							% check target reached
						else 
							% compute trajectory step
							[nextX,nextY,rotationToDo,lenToDo,direct,startHeading,forward] = ComputeNextStepToTarget(carto,robot.GetHardPosX,robot.GetHardPosY,robot.GetHardAngle,targetX,targetY,plotOn);
							if (forward==0)
								printf("no path found. \n")
								issue=true;
								pause
								return
							endif
							printf("Next X:%d Y:%d . \n",nextX,nextY)
							% move
%							ready=false;
%						while (ready==false && issue==false)
%							printf("robot goto X:%d Y:%d . \n",nextX,nextY)
%							ready=yes_or_no(" ok ?")
%						end 
%{
							inp=eval(input("enter 0 to stop, 1 to resume ","i"));
							if (inp==0)
								return
							endif
%}
							if (direct==false)
								[rotationToDo,lenToDo]=ComputeMoveToDo(robot.GetHardPosX,robot.GetHardPosY,robot.GetHardAngle,nextX,nextY)
							endif
%{
							if (rotationToDo!=0)
								printf("align robot:%d  \n",robot.GetNorthOrientation()-rotationToDo)
								robot.NorthAlign(robot.GetNorthOrientation()-rotationToDo);    % align robot such that it will be in the same orientation as for learning phase
								WaitFor=WaitNorthAlign;
								retCode=WaitForRobot(robot,WaitFor);
								if (retCode==0)
									aligned=true;
									else
									[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
									if action=="stop"
										return
									endif
								endif
							endif
%}			
							robot.Move(rotationToDo,lenToDo*forward) 		 % len sent in cm
%							particles=ResampleParticles(plotOn,particles);
							particles=MoveParticles(rotationToDo,lenToDo,plotOn,particles);
							WaitForRobot(robot,WaitMove);
	%						robot.UpdateRobotStatus();
	%						WaitFor=WaitInfo;
	%						retCode=WaitForRobot(robot,WaitFor);
							if (retCode>=9)  
								[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
							if (retCode==1)
								printf("incompleted move due to obstacle. \n")
							endif														
							if (retCode==2)
								printf("no move at all. \n")
							endif
							robot.ValidHardPosition();
							newX=robot.GetHardPosX();
							newY=robot.GetHardPosY();
							newH=robot.GetHardAngle();
							newProb=robot.GetCurrentLocProb();
							prob = TestLocationEchoConsistancy(robot,carto,newX,newY,newH);
							printf("test consistancy prob: %d \n",prob)
							[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(newX,newY,prob,plotOn,particles);
							printf("detX: %d \n",detX)
							printf("detY: %d \n",detY)
							printf("detH: %d \n",detH)
							robot.SetPosX(detX);
							robot.SetPosY(detY);
							robot.SetHeading(detH);
							robot.SetCurrentLocProb(prob);
							printf("update hard robot. \n")
							robot.UpdateHardRobotLocation();	
							WaitFor=WaitUpdate;
							retCode=WaitForRobot(robot,WaitFor);
							if (retCode!=0)
								[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
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
							particles=ResampleParticles(plotOn,particles);
							if ((mod(newH+robot.GetNorthOrientation(),360)/shiftNorthXOrientation)>1.2)  % to much difference between robot calculation and NO
								printf("heading inconsitancy :%d NO:%d . \n",newH,robot.GetNorthOrientation())
%								headingIssue=true
							endif
%							robot.SetAlpha(robot.GetHardAngle())								
						endif
				else
					printf("robot location inconsistency X:%d expected:%d Y:%d expected:%d orientation:%d expected:%d. \n",robot.GetHardPosX,newX,robot.GetHardPosY,newY,robot.GetHardAngle,newAngle)
					issue=1;
				endif

	end