function [weight] = TestLocationEchoConsistancy(robot,carto,posX,posY,heading)
%{
This function does 4 echos measurments based on the (X,Y) cartography referential
Parameters are:
	robot: the java object that is used to drive the physical robot
	carto: the cartography matrix
	posX: a vector containing a list of position regarding X axis
	posY: a vector containing a list of position regarding Y axis (the number of values posY must be the same as posX)
	heading: is the robot orientation
	
The aim of this function is to provide a weight for each position (x,y)
Compare echos with theoretical distances to determine the weight
In simulation mode the echoes are simulated based on theoritical distances and noised added
The higher is the weight the most likely is the position
	Query the cartography obstacle distances (QueryCartoEchosProjection) 
	Align the servo motor with th X axis 
	Ping front and back to get distances
	Align the servo motor with th Y axis 
	Ping front and back to get distances
	Compute and return the weight
%}
heading=mod(heading,2*pi());
[nbLine,nbCol]=size(posX);                  % get the vector size
WaitServoAlign=robot.servoAlignEnd;          % define the wait for servo motor aligment value
WaitPingFB=robot.pingFBEnd;                  % define the wait for ping code value
callFrom=1;          % define the main function for the waitFor function
coefWeight=1;        % reserved to adjust weight if necessary
simuParam1=10;       % coefficient used to determine the noise in simulation mode - Increasing the value will increase the noise
simulationMode=90;    % used to identify the simulation mode  (retCode of waitFor function)
echo=[];
pts=[posX;posY]';   % create matric of points  (x,y)
printf ("Check echoConsistancy for points:");
for i=1:nbCol
	printf(" (%d,%d)",pts(i,1),pts(i,2)); % print the points list
endfor
pts=repmat(pts,1,2);   % create matric of points (2,nbCol) (x,y)
printf (" *** ");
printf(ctime(time()));
weight=[];

% create matrix to project obstacle on (X,Y) absolute referential
projection=[];
for i=1:nbCol         
	[p0,p1,p2,p3] = QueryCartoEchosProjection(carto,posX(i),posY(i));  % get the theorical points from the cartography
	projection = [projection;[p0,p1,p2,p3]];                           % create a matrix - one line for each points
endfor
projection=[projection(:,1),projection(:,4),projection(:,5),projection(:,8)];  % keep the significative values following (+X,+Y,-X,-Y)
nnzProj=projection==0;
nnzProj=nnzProj==0;                    % a matrix that contain 1 for each non zero of projection
nbnnz=(sum((projection==0)==0,2))';    % get a vector containing the number of non zero in each line
% ping echo
if (heading>=0 && heading <=(pi()/2))
	a2=pi()/2-heading;
	a1=pi()-heading;
endif
if (heading>pi()/2 && heading <=(pi()))
	a1=pi()-heading;
	a2=3*pi()/2-heading;
endif
if (heading>pi() && heading <=(3*pi()/2))
	a2=3*pi()/2-heading;
	a1=2*pi()-heading;
endif
if (heading>3*pi()/2 && heading <=2*pi())
	a2=5*pi()/2-heading;
	a1=2*pi()-heading;
endif
printf(" servoAlign: %d & %d **** ",a1*180/pi(),a2*180/pi()); % print the points list
printf(ctime(time()))
robot.RobotAlignServo(floor(a1*180/pi()));     % ask the robot to align with X axis taking into account the current heading
WaitFor=WaitServoAlign;                       % define the waiting for value for the wairFor function
retCode=WaitForRobot(robot,WaitFor);			% wait for robot feedback
if (retCode!=0)                                % analyse the non zero return code
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
robot.PingEchoFrontBack();             %  ask the robot to ping echo front and back
WaitFor=WaitPingFB;                        % define the waiting for value for the wairFor function
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot feedback
if (retCode!=0)                           % analyse the non zero return code
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
if retCode==simulationMode         % test if we are in simulation mode

else
	if (heading>=0 && heading <=(pi()))
	echo1=robot.GetScanDistFront(0);   % if normal mode get the robot echoes
	echo2=robot.GetScanDistBack(0);    % if normal mode get the robot echoes
	endif
	if (heading>pi())
	echo1=robot.GetScanDistBack(0);   % if normal mode get the robot echoes
	echo2=robot.GetScanDistFront(0);    % if normal mode get the robot echoes
	endif
endif

robot.RobotAlignServo(floor(a2*180/pi())); % ask the robot to align with Y axis taking into account the current heading
WaitFor=WaitServoAlign;                       % define the waiting for value for the wairFor function
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
if (retCode!=0)                     % analyse the non zero return code
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
robot.PingEchoFrontBack();                 %  ask the robot to ping echo front and back
WaitFor=WaitPingFB;                         % define the waiting for value for the wairFor function
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
if (retCode!=0)                              % analyse the non zero return code
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
if retCode==simulationMode                 % test if we are in simulation mode
	simuParam2=floor(rand*nbCol+1);  % to choose wich value will be the simulation reference
	printf ("simulator choice for echo simulation is the %d position *** ",simuParam2);
	printf(ctime(time()));
	% compute the 4 theoretical distances with the obstacles
	echo=[projection(simuParam2,1)-posX(simuParam2),projection(simuParam2,2)-posY(simuParam2),projection(simuParam2,3)-posX(simuParam2),projection(simuParam2,4)-posY(simuParam2)];
	echo=normrnd(abs(echo),abs(echo)/simuParam1);  % add some noise to the theoretical values
	printf("simulation echo +X:%d +Y:%d -X:%d -Y:%d *** ",echo(1),echo(2),echo(3),echo(4));
	printf(ctime(time()));
	% mode simulation
else
	if ((heading>=0 && heading <=(pi()/2)) || (heading>3*pi()/2))
		echo3=robot.GetScanDistFront(0);   % if normal mode get the robot echoes
		echo4=robot.GetScanDistBack(0);    % if normal mode get the robot echoes
	endif
		if ((heading>=pi()/2 && heading <=(3*pi()/2)) )
		echo3=robot.GetScanDistBack(0);   % if normal mode get the robot echoes
		echo4=robot.GetScanDistFront(0);    % if normal mode get the robot echoes
	endif
	echo=[echo3,echo1,echo4,echo2];         % +X +Y -X -Y
	printf("real echo +X:%d +Y:%d -X:%d -Y:%d *** ",echo(1),echo(2),echo(3),echo(4));
	printf(ctime(time()));
endif
% then compute distances for each positions
echo=[echo(1),echo(2),-echo(3),-echo(4)];
echo=repmat(echo,nbCol,1);            % create matrix by duplicating echo vector
% compute the weight taking into account the number of "references" (non zero projections)
% the higher is the nb references the higher is the weight
projection
echo
pts
weight=max(1,100-((sqrt(sum((abs(projection-echo-pts).*nnzProj).^2,2))')./(nbnnz))*coefWeight);


endfunction