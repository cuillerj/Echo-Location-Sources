function [weight] = TestLocationEchoConsistancy(robot,carto,posX,posY,heading)
% en cours de ddev a partir de la v0 sous forme matricielle pour ne pas rescanner
[nbLine,nbCol]=size(posX);
WaitServoAlign=robot.servoAlignEnd;          % set servo motor in 
WaitPingFB=robot.pingFBEnd;
callFrom=1;          % to identify the main function
coefWeight=1;
%simuParam1=30;
simuParam3=10;
simulationMode=90;        % retCode in simulation mode
%simuParam2=1;
%posX=[55,115,180,200]
%posY=[55,55,120,120]
%echo=[190,50,10,10]
echo=[];
[nbLine,nbCol]=size(posX);
pts=[posX;posY]';   % create matric of points  (x,y)
printf ("EchoConsistancy for points:");
for i=1:nbCol
	printf(" (%d,%d)",pts(i,1),pts(i,2));
endfor
pts=repmat(pts,1,2);   % create matric of points (2,nbCol) (x,y)
printf (" *** ");
printf(ctime(time()));
weight=[];

% create matrix to project obstacle on (X,Y) absolute referential
projection=[];
for i=1:nbCol         
	[p0,p1,p2,p3] = QueryCartoEchosProjection(carto,posX(i),posY(i));
	projection = [projection;[p0,p1,p2,p3]]; 
endfor
projection=[projection(:,1),projection(:,4),projection(:,5),projection(:,8)];  % keep the 4 significative rows
nnzProj=projection==0;
nnzProj=nnzProj==0;
nbnnz=(sum((projection==0)==0,2))';    % get a vector containing the number of non zero in each line
% ping echo
robot.RobotAlignServo(floor(90-heading));
WaitFor=WaitServoAlign;
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
if (retCode!=0)
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
robot.PingEchoFrontBack();
WaitFor=WaitPingFB;
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
if (retCode!=0)
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
if retCode==simulationMode         % simulation mode
% simuation mode
%	simuParam2=floor(rand*nbCol+1)  % to choose wich value will be the simulation reference
%	echo=[projection(simuParam2,1)-posX(simuParam2),projection(simuParam2,2)-posY(simuParam2),projection(simuParam2,3)-posX(simuParam2),projection(simuParam2,4)-posY(simuParam2)]
%	echo=normrnd(echo,abs(echo)/simuParam3)

else
	echo0=robot.GetScanDistFront(0);
	echo2=robot.GetScanDistBack(0);
endif

robot.RobotAlignServo(floor(180-heading));
WaitFor=WaitServoAlign;
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
if (retCode!=0)
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
robot.PingEchoFrontBack();
WaitFor=WaitPingFB;
retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
if (retCode!=0)
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
if retCode==simulationMode
	simuParam2=floor(rand*nbCol+1);  % to choose wich value will be the simulation reference
	printf ("simulator choice for echo simulation is the %d position *** ",simuParam2);
	printf(ctime(time()));
	echo=[projection(simuParam2,1)-posX(simuParam2),projection(simuParam2,2)-posY(simuParam2),projection(simuParam2,3)-posX(simuParam2),projection(simuParam2,4)-posY(simuParam2)];
	echo=normrnd(echo,abs(echo)/simuParam3);
	printf("simulation echo +X:%d +Y:%d -X:%d -Y:%d *** ",echo(1),echo(2),echo(3),echo(4));
	printf(ctime(time()));
	% mode simulation
else
	echo1=robot.GetScanDistFront(0);
	echo3=robot.GetScanDistBack(0);
	echo=[echo0,echo1,echo2,echo3];
	printf("real echo +X:%d +Y:%d -X:%d -Y:%d *** ",echo(1),echo(2),echo(3),echo(4));
	printf(ctime(time()));
endif
% then compute distances for each positions
echo=repmat(echo,nbCol,1);

weight=max(1,100-((sqrt(sum((abs(projection-echo-pts).*nnzProj).^2,2))')./(nbnnz))*coefWeight);


endfunction