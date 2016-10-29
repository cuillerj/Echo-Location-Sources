 function [] = TestTrajectoryControl(simulationMode)
 cpu1=cputime();
   if (exist("simulationMode"))
  else
    simulationMode=1;
  endif
 %simulationMode=1;  % to set simulation 1 on 0 off
[robot,carto,img,headingNOrthXOrientation,zonesXY,all_theta] = InitOctaveRobot();
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
newX=55;
newY=55;
newH=0;
newProb=100;
probExpectedMoveOk=50;
probHardMoveOk=50;
shiftNO=0;
printf("robot location is X:%d Y:%d orientation: %d. prob:%d *** ",newX,newY,newH,newProb);
printf(ctime(time()));
%particles = CreateLocatedParticles(carto,newX,newY,newH,newProb,particlesNumber,plotOff);
particles = CreateLocatedParticles(carto,img,-1,-1,-1,0,particlesNumber,plotOn);
spaceNO=SpaceNorthOrientation(zonesXY,newX,newY);  % 
trajectory=[newX,newY];
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
targetX=370;
targetY=250;
targetH=0;;
newTarget=1;
printf("robot target is X:%d Y:%d orientation: %d. *** ",targetX,targetY,targetH)
printf(ctime(time()))
%

robot.SetPosX(posX);
robot.SetPosY(posY);
robot.SetHeading(heading);
robot.SetCurrentLocProb(locProb);
printf("update hard robot. *** ")
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
			robot.SetPosX(floor(newX));
			robot.SetPosY(floor(newY));
			robot.SetHeading(floor(newH));
			robot.SetCurrentLocProb(newProb);
			printf("update hard robot.. *** ");
			printf(ctime(time()));
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
			printf("hardPosX:%d hardPosY:%d *** ",robot.GetHardPosX,robot.GetHardPosY);
			printf(ctime(time()))
			
while (issue==false && targetReached==false)

	rotation=0;

			if ((abs(robot.GetHardPosX-floor(newX))<=1  && abs(robot.GetHardPosY-floor(newY))<=1 && abs(robot.GetHardHeading-floor(newH))<=10))
						if (QueryCartoAvailability(carto,newX,newY,newH*pi()/180,true)==false)
							currentPositionIssue=true;
						endif
						if ((targetX-robot.GetHardPosX)^2 + (targetY-robot.GetHardPosY)^2 <=225)
							targetReached=true;
							% check target reached
						else 
							% compute trajectory step
							[nextX,nextY,rotationToDo,lenToDo,direct,forward] = ComputeNextStepToTarget(carto,robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading,targetX,targetY,newTarget,plotOn);
							newTarget=0;
							if (forward==0)
								printf("no path found. *** ")
								printf(ctime(time()))
								issue=true;
								pause
								return
							endif
							printf("Next step is X:%d Y:%d . *** ",nextX,nextY)
							printf(ctime(time()))

%							if (direct==false)
								[rotationToDo,lenToDo]=ComputeMoveToDo(robot.GetHardPosX,robot.GetHardPosY,robot.GetHardHeading,nextX,nextY,forward);
%							else
%								nextX=targetX;         % straight move possible next step is target
%								nextY=targetY;			% straight move possible next step is target
%							endif
							printf("rotation:%d distance:%d *** ",rotationToDo,lenToDo);
							printf(ctime(time()));
							saveTargetHeading=robot.GetHeading()+rotationToDo;  % replace rotation by rotationToDo le 28/09
							if (rotationToDo!=0)
								printf("NO rotate robot NO: %d rotation:%d  *** ",robot.GetNorthOrientation(),rotationToDo)
								printf(ctime(time()))
								robot.RobotNorthRotate(mod(rotationToDo+360,360));    % rotation based on north orientation
								WaitFor=WaitNorthAlign;
								retCode=WaitForRobot(robot,WaitFor);
								if (retCode==0)
									aligned=true;
									else
									[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
									if action=="stop"
										return
									endif
								endif
							endif
							robot.SetHeading(mod(360+robot.GetHardHeading(),360));
							robot.Move(0,lenToDo); 		 % len sent in cm
							lastParticles=particles;          % to be able to recover in case of move failure
							particles=MoveParticles(rotationToDo,lenToDo,img,plotOff,particles);
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
							probExpectedMoveOk=50;
							if (retCode==robot.moveKoDueToSpeedInconsistancy )
								newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
								printf("incompleted DueToSpeedInconsistancy expected: %d actual:%d %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
								printf(ctime(time()))
								particles=MoveParticles(rotationToDo,newLenToDo,img,plotOff,lastParticles);
								probExpectedMoveOk=25;
							endif	
							if (retCode==robot.moveKoDueToObstacle )
								newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
								printf("incompleted move due to obstacle expected: %d actual:%d  %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
								printf(ctime(time()))
								particles=MoveParticles(rotationToDo,newLenToDo,img,plotOff,lastParticles);
								probExpectedMoveOk=25;
							endif														
							if (retCode==robot.moveUnderLimitation)
								printf("no move moveUnderLimitation. *** ")
								printf(ctime(time()))
								particles=lastParticles;
								probExpectedMoveOk=1;
							endif
							robot.ValidHardPosition();
							newX=[nextX,robot.GetHardPosX()];
							newY=[nextY,robot.GetHardPosY()];
							newH=[mod(360+robot.GetHardHeading(),360)];
							[available,retCode]=QueryCartoAvailability(carto,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading()*pi()/180,1);
							if(available==false)
								probHardMoveOk=1;  % the hard position is theoriticaly not possible
								printf("the hard position is theoriticaly not possible. *** x:%d y:%d h:%d ",robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardHeading())
								printf(ctime(time()))
							endif
							newProb=[probExpectedMoveOk,probHardMoveOk];
							if (probExpectedMoveOk >= probHardMoveOk)
								spaceNO=SpaceNorthOrientation(zonesXY,newX(1),newY(1))-shiftNO-robot.GetNorthOrientation();  
							else
								spaceNO=SpaceNorthOrientation(zonesXY,newX(2),newY(2))-shiftNO-robot.GetNorthOrientation();
							endif
							headingNO=mod(360+spaceNO+shiftNO,360); 
%							prob=newProb;
							[line,col]=size(newX);
							weightEcho=[];
							alpha1=mod(newH+360,360);
%							alpha2=mod(headingNO+360,360);
							alpha3=mod(saveTargetHeading+360,360);
							if (simulationMode==1)
								averageH=floor(mod((alpha1+alpha3)/2,360));
							else
								alpha2=mod(headingNO+360,360);
								averageH=floor(mod((alpha1+alpha2+alpha3)/3,360));
							endif
							printf ("Heading hard H:%d  NO:%d Theoretical H:%d  Average:%d SpaceNO:%d RobotNO:%d *** ",newH,headingNO,saveTargetHeading,averageH,spaceNO,robot.GetNorthOrientation());
							printf(ctime(time()))
							weightEcho = TestLocationEchoConsistancy(robot,carto,newX,newY,averageH*pi()/180);
							printf("echo consistancy prob:");
							for i=1:col	
								printf(" %d ",weightEcho(i));
							endfor;		
							printf("*** ");
							printf(ctime(time()));
							[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(newX,newY,weightEcho,img,plotOn,particles);
							printf("detX: %d ",detX)
							printf(" detY: %d ",detY)
							printf(" detH: %d ",detH)
							printf(ctime(time()))
							trajectory=[trajectory;[detX,detY]];
							newX=detX;
							newY=detY;
							newH=detH;
							robot.SetPosX(detX);
							robot.SetPosY(detY);
							robot.SetHeading(detH);
%							printf("det prob: %d *** ",prob);
%							printf(ctime(time()));
%							robot.SetCurrentLocProb(prob(1));
							printf("update hard robot... *** ")
							printf(ctime(time()))
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
								endif
							endif							
							particles=ResampleParticles(img,plotOff,particles);
							if (mod(360+newH,360)-mod(360+headingNO,360)>15)  % to much difference between robot calculation and NO
								printf("inconsitancy heading:%d headingNO:%d NO:%d SpaceNO:%d *** ",newH,headingNO,robot.GetNorthOrientation(),spaceNO)
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
AStarShowStep(trajectory,"actual trajectory");
endfunction
hold off