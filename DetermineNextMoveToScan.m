function [rotation,moveDist] = DetermineNextMoveToScan(robot,plotOn)
	[param,value,found]=ApeRobotCommonDefine("nbPulse");
	nbPingByScan=value-1;
	[param,value,found]=ApeRobotCommonDefine("frontLenght");
	frontLenght=value;
	[param,value,found]=ApeRobotCommonDefine("securityLenght");
	securityLenght=(value+frontLenght)*1.1;
	shiftAngle=pi()/(nbPingByScan);
	minDistance=60;
	validMoveList=[];
	maxDist=200;
	if (plotOn)
		plotEchoScanRobot(robot);
	endif
for (i=0:nbPingByScan-1)
	angleR=(i)*shiftAngle;
	if (i==0)
		echo0=min(robot.GetScanDistBack(nbPingByScan-1),maxDist);
	else
		echo0=min(robot.GetScanDistFront(i-1),maxDist);
	endif
	echo1=min(robot.GetScanDistFront(i),maxDist);
	echo2=min(robot.GetScanDistFront(i+1),maxDist);
	coefEcho1=unifrnd(0.5,0.6);
	if (echo0>=minDistance && echo1>=minDistance && echo2>=minDistance)
		validMoveList(i+1,1)=2*echo1+echo0+echo2;
		validMoveList(i+1,2)=min([echo0-securityLenght,echo1*coefEcho1,echo1-securityLenght,echo2-securityLenght]);
		validMoveList(i+1,3)=angleR;
	endif
	if (i==0)
		echo0=min(robot.GetScanDistFront(nbPingByScan-1),maxDist);
	else
		echo0=min(robot.GetScanDistBack(i-1),maxDist);
	endif
	echo1=min(robot.GetScanDistBack(i),maxDist);
	echo2=min(robot.GetScanDistBack(i+1),maxDist);
	if (echo0>=minDistance && echo1>=minDistance && echo2>=minDistance)
		validMoveList(i+1,4)=2*echo1+echo0+echo2;
		validMoveList(i+1,5)=min([echo0-securityLenght,echo1*coefEcho1,echo1-securityLenght,echo2-securityLenght]);
%		validMoveList(i+1,6)=angleR;
	endif
end
%validMoveList
[maxValues,idxMax]=max(validMoveList)
maxFront=maxValues(1);
maxBack=maxValues(4);
if (maxFront>=maxBack)
	moveDist=validMoveList(idxMax(1),2);
	rotation=3*pi()/2+validMoveList(idxMax(1),3);
else
	moveDist=-validMoveList(idxMax(4),5);
	rotation=3*pi()/2+validMoveList(idxMax(4),3);
endif
rotation=mod(rotation,2*pi());
return
endfunction