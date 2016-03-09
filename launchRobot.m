cd ..
javaaddpath ('C:\Users\jean\Documents\Donnees\eclipse\RobotServer\bin\robot.jar');
% create and start robot
% load matrix
shiftNorthXOrientation=268; % shift between north and X direction in degres
setupPath;
robot=robotJava;
methods=javamethods(robot);
robot.SetTraceFileOn(1);    % route console to trace file
robot.LaunchBatch();          % call java method to start batch
load "cartoCuisine.txt";
load all_theta;
nbLocPossibility=size(all_theta,1);
predLocMatrix=InitLocMatrix(all_theta);
% load "zonesXY.txt"
j=1;
issue=0;
targetReached=false;
ready=false;
nbPred=5;
while (ready==false)
	ready=yes_or_no(" robot ready to go ?")
end
% compute target location
[targetX,targetY,targetAngle]=ComputeTargetLocation(robot);
printf("robot target is X:%d Y:%d orientation: %d. \n",targetX,targetY,targetAngle)
sleep(1);
% loop till target reached
	% localize robot
robot.NorthAlignRobot(shiftNorthXOrientation);
count=0;
printf("north aligning \n")
aligned=false;
while (aligned==false)
	status=robot.GetOctaveRequestPending();
	if (status==false)
		aligned=true;
	endif
	if (mod(count,10)==0)
		printf("robot status: %d. \n",status);
	endif
	count =count+1;
	sleep(1)
end

while (issue==0 && targetReached==false)
printf("echo loc \n")
%	[echoX,echoY,echoAngle,echoProb]=EchoLocalizeRobot(robot,nbLocPossibility,nbPred);
	orientation=mod(robot.GetNorthOrientation+360-shiftNorthXOrientation,360)
	[echoX,echoY,echoAngle,echoProb]=EchoLocalizeRobotWithRotation(robot,nbLocPossibility,nbPred,orientation);
	for i=1:nbPred
		printf("robot echo location is X:%d Y:%d orientation:%d with %d%% probability. \n",echoX(i),echoY(i),echoAngle(i),echoProb(i))
	endfor
	[newX,newY,newAngle,newProb,predLocMatrix]=DetermineRobotLocation(robot,robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHardAngle,echoX,echoY,echoAngle,echoProb,predLocMatrix,shiftNorthXOrientation);
	robot.SetPosX(newX);
	robot.SetPosY(newY);
	robot.SetAlpha(newAngle);
	robot.SetCurrentLocProb(newProb);
	robot.UpdateHardRobotLocation();
	printf("determined location is X:%d Y:%d orientation:%d with %d%% probability. \n",newX,newY,newAngle,newProb)
	sleep(2);
	if (robot.GetHardPosX==newX && robot.GetHardPosY==newY && robot.GetHardAngle==floor(newAngle))
			if (abs(targetX-robot.GetHardPosX)<=20 && abs(targetY-robot.GetHardPosY)<=20)
				targetReached=true;
				% check target reached
			else 
				% compute trajectory step
				[nextX,nextY,nextAngle] = ComputeNextStepToTarget(targetX,targetY,targetAngle)
				% move
				ready=false;
while (ready==false)
	printf("robot goto X:%d Y:%d . \n",nextX,nextY)
	ready=yes_or_no(" ok ?")
end 
				[rotationToDo,lenToDo]=ComputeMoveToDo(robot.GetHardPosX,robot.GetHardPosY,robot.GetHardAngle,nextX,nextY);
				robot.Move(rotationToDo,lenToDo*10);  % len sent in mm
				while (robot.GetRunningStatus()!=105)
				sleep(1);
				end
				robot.SetPosX(robot.GetHardPosX());
				robot.SetPosY(robot.GetHardPosY());
				robot.SetAlpha(robot.GetHardAngle());
			endif
	else
		printf("robot location inconsistency X:%d expected:%d Y:%d expected:%d orientation:%d expected:%d. \n",robot.GetHardPosX,newX,robot.GetHardPosY,newY,robot.GetHardAngle,newAngle)
		issue=1;
	endif
end

