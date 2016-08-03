function [weight] = TestLocationEchoConsistancyv0(robot,carto,posX,posY,heading)
[p0,p1,p2,p3] = QueryCartoEchosProjection(carto,posX,posY);  % 
d0=d1=d2=d3=[-30];
WaitServoAlign=robot.servoAlignEnd;          % set servo motor in 
WaitPingFB=robot.pingFBEnd;
callFrom=1;          % to identify the main function
coefProb=1.5;
nb=0;
if (p0!=0 || p2!=0)
	robot.RobotAlignServo(90-heading);
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
	if retCode==90
		echo0=p0(1)-posX;
		echo2=posX-p2(1);
		echo0=normrnd(echo0,echo0/4);
		echo2=normrnd(echo2,echo2/4);
	% mode simulation
	else
		echo0=robot.GetScanDistFront(0)
		echo2=robot.GetScanDistBack(0)
	endif
endif
if (p0(1)!=0)
	d0=p0(1)-(posX+echo0);
	nb++;
endif
if (p2(2)!=0)
	d2=p2(1)-(posX-echo2);
	nb++;
endif


if (p1!=0 || p3!=0)
	robot.RobotAlignServo(180-heading);
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
	if retCode==90
		echo1=p1(2)-posY;
		echo3=posY-p3(2);
		echo1=normrnd(echo1,echo1/4);        % add some noise
		echo3=normrnd(echo3,echo3/4);       % add some noise
	% mode simulation
	else
		echo1=robot.GetScanDistFront(0);
		echo3=robot.GetScanDistBack(0);

	endif
	if (p1(1)!=0)
		d1=p1(2)-(posY+echo1);
		nb++;
	endif
	if (p3(2)!=0)
		d3=p3(2)-(posY-echo3);
		nb++;
	endif


endif
if (nb!=0)
	printf("delta echo theoratical distance +X:%d -X:%d +Y:%d -Y:%d nb values in account:%d *** ",d0,d1,d2,d3,nb)
	printf(ctime(time()));
	weight=floor(max(1,100-(sqrt(d0.^2+d1.^2+d2.^2+d3.^2)/(nb))*coefProb));
else
	weight=1;
endif
endfunction