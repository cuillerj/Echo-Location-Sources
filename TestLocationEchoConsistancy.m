function [prob] = TestLocationEchoConsistancy(robot,carto,posX,posY,heading)
[p0,p1,p2,p3] = QueryCartoEchosProjection(carto,posX,posY);
d0=d1=d2=d3=0;
WaitServoAlign=robot.servoAlignEnd;
WaitPingFB=robot.pingFBEnd;
callFrom=1;          % to identify the main function
coefProb=2;
nb=0;
if (p0!=0 || p2!=0)
	robot.RobotAlignServo(90-heading);
	WaitFor=WaitServoAlign;
	retCode=WaitForRobot(robot,WaitFor);			% wait fo robot
		if (retCode!=0)
			[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
			if action=="stop.."
				return
			endif
			if action=="break."
				break
			endif
			if action=="resume"
						paus
			endif
		endif
	robot.PingEchoFrontBack();
	WaitFor=WaitPingFB;
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
						paus
			endif
		endif
	echo0=robot.GetScanDistFront(0)
	echo2=robot.GetScanDistBack(0)
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
			[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
			if action=="stop"
				return
			endif
			if action=="break"
				break
			endif
			if action=="resume"
						paus
			endif
		endif
	robot.PingEchoFrontBack();
	WaitFor=WaitPingFB;
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
						paus
			endif
		endif
	echo1=robot.GetScanDistFront(0)
	echo3=robot.GetScanDistBack(0)
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
	prob=floor(max(1,100-(sqrt(d0^2+d1^2+d2^2+d3^2)/(nb^2))*coefProb));
else
	prob=1;
endif
endfunction