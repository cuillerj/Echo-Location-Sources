function [weight,retValue,echo,quality] = TestLocationEchoConsistancyVsDB(robot,carto,posX,posY,heading,parametersNameList)
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
WaitInfo=robot.robotInfoUpdated; 
%{
lastBNOmode=robot.BNOMode;
[param,value,number]=GetParametersValueByName(robot,"CompasMode",parametersNameList);
CompasMode=value;
if (lastBNOmode!=CompasMode)
	robot.setBNOMode(CompasMode);
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
		endif

		while (robot.BNOMode!=CompasMode)
			printf("wait for BNOMode");
			robot.setBNOMode(CompasMode);
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
			endif
			sleep(2);
		end
				%}
[param,value,number]=GetParametersValueByName(robot,"tileSize1",parametersNameList);
tileSize=value/10;
countPos=zeros(size(posX));
yFrontEchoRef=zeros(size(posX));
yBackEchoRef=zeros(size(posX));
stdRefFrontY=zeros(size(posX));
stdRefBackY=zeros(size(posX));
countRefY=zeros(size(posX));
xFrontEchoRef=zeros(size(posX));
xBackEchoRef=zeros(size(posX));
stdRefFrontX=zeros(size(posX));
stdRefBackX=zeros(size(posX));
countRefX=zeros(size(posX));
	if (heading>=0 && heading <=(pi()/2))
		a2=pi()/2-heading;
		a1=pi()-heading;
		Xoriented=1;
		Yoriented=1;
	endif
	if (heading>pi()/2 && heading <=(pi()))
		a1=pi()-heading;
		a2=3*pi()/2-heading;
		Xoriented=-1;
		Yoriented=1;
	endif
	if (heading>pi() && heading <=(3*pi()/2))
		a2=3*pi()/2-heading;
		a1=2*pi()-heading;
		Xoriented=-1;
		Yoriented=-1;
	endif
	if (heading>3*pi()/2 && heading <=2*pi())
		a2=5*pi()/2-heading;
		a1=2*pi()-heading;
		Xoriented=1;
		Yoriented=-1;
	endif
for (i=1:size(posX,2))
	distBeetwenPosReference=robot.GetClosestReferenceEcho(posX(i),posY(i),tileSize,0);
	if (robot.echoClosestCount!=0)        % check we find an echo reference
		countPos(i)++;
	endif
		yFrontEchoRef(i)=robot.echoClosestRefDistFront+(-posY(i)+robot.echoClosestRefY)*Yoriented;
		yBackEchoRef(i)=robot.echoClosestRefDistBack-(-posY(i)+robot.echoClosestRefY)*Yoriented;
		stdRefFrontY(i)=robot.echoClosestStdFront;
		stdRefBackY(i)=robot.echoClosestStdBack;
		countRefY(i)=robot.echoClosestCount;
		distBeetwenPosReference=robot.GetClosestReferenceEcho(posX(i),posY(i),tileSize,90);
	if (robot.echoClosestCount!=0)        % check we find an echo reference
		countPos(i)++;
	endif
		xFrontEchoRef(i)=robot.echoClosestRefDistFront+(-posX(i)+robot.echoClosestRefX)*Xoriented;
		xBackEchoRef(i)=robot.echoClosestRefDistBack-(-posX(i)+robot.echoClosestRefX)*Xoriented;
		stdRefFrontX(i)=robot.echoClosestStdFront;
		stdRefBackX(i)=robot.echoClosestStdBack;
		countRefX(i)=robot.echoClosestCount;
endfor
	if (sum(countPos)==0)           % no available echo reference
		weight=[];
		retValue=-1;
		echo=[];
		return;
	endif
	heading=mod(heading,2*pi());
	[nbLine,nbCol]=size(posX);                  % get the vector size
	WaitServoAlign=robot.servoAlignEnd;          % define the wait for servo motor aligment value
	WaitPingFB=robot.pingFBEnd;                  % define the wait for ping code value
	callFrom=1;          % define the main function for the waitFor function
	coefWeight=1;        % reserved to adjust weight if necessary
	simuParam1=20;       % coefficient used to determine the noise in simulation mode - Increasing the value will increase the noise
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
	retValue=0;
	% create matrix to project obstacle on (X,Y) absolute referential

       
	projection=abs([xFrontEchoRef;yFrontEchoRef;xBackEchoRef;yBackEchoRef]);  % keep the significative values following (+X,+Y,-X,-Y)


	% ping echo

	printf(" servoAlign: %d & %d **** ",a1*180/pi(),a2*180/pi()); % print the points list
	printf(ctime(time()))
	robot.RobotAlignServo(round(a1*180/pi()));     % ask the robot to align with X axis taking into account the current heading
	WaitFor=WaitServoAlign;                       % define the waiting for value for the wairFor function
	retCode=WaitForRobot(robot,WaitFor);			% wait for robot feedback
	if (retCode!=0 && retCode!=simulationMode )                                % analyse the non zero return code
		[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
		retValue=-1;
		return
	endif
	robot.PingEchoFrontBack();             %  ask the robot to ping echo front and back
	WaitFor=WaitPingFB;                        % define the waiting for value for the wairFor function
	retCode=WaitForRobot(robot,WaitFor);			% wait fo robot feedback
	if (retCode!=0 && retCode!=simulationMode)                           % analyse the non zero return code
		[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
		retValue=-1;
		return
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
	robot.RobotAlignServo(round(a2*180/pi())); % ask the robot to align with Y axis taking into account the current heading
	WaitFor=WaitServoAlign;                       % define the waiting for value for the wairFor function
	retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
	if (retCode!=0 && retCode!=simulationMode)                     % analyse the non zero return code
		[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
		retValue=-1;
		return
	endif
	robot.PingEchoFrontBack();                 %  ask the robot to ping echo front and back
	WaitFor=WaitPingFB;                         % define the waiting for value for the wairFor function
	retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
	if (retCode!=0 && retCode!=simulationMode)                              % analyse the non zero return code
		[issue,action]=analyseRetcode(robot,retCode,WaitFor,callFrom);
		retValue=-1;
		return
	endif
	retValue=0;
	if retCode==simulationMode                 % test if we are in simulation mode
		simuParam2=floor(rand*nbCol+1);  % to choose wich value will be the simulation reference
		printf ("simulator choice for echo simulation is the %d position *** ",simuParam2);
		printf(ctime(time()));
		% compute the 4 theoretical distances with the obstacles
		echo=[projection(1,simuParam2),projection(2,simuParam2),projection(3,simuParam2),projection(4,simuParam2)];
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
	echo=[echo(1),echo(2),echo(3),echo(4)];
	echo=repmat(echo,nbCol,1);        % create matrix by duplicating echo vector
	% compute the weight taking into account the number of "references" (non zero projections)
	projection
	echo'
	quality=min(sum(abs(projection-echo'),1))
	weight=(sum(sum(abs(projection-echo'),1))-sum(abs(projection-echo'),1)).*countPos;
	minMatrixValue=ones(size(weight)).*0.001
	weight=max(weight/sum(weight),minMatrixValue);
%	weight=max(1,100-((sqrt(sum((abs(projection-echo).*nnzProj).^2,2))')./(nbnnz))*coefWeight);

echo=echo(1,:);
endfunction