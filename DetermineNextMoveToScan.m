function [rotation,moveDist] = DetermineNextMoveToScan(robot,plotOn)
	[param,value,found]=ApeRobotCommonDefine("nbPulse");
	nbPingByScan=value-1;
	shiftAngle=pi()/(nbPingByScan);
	minDistance=60;
	validMoveList=[];
	if (plotOn)
		plotEchoScanRobot(robot);
	endif
for (i=0:nbPingByScan-1)
	angleR=(i)*shiftAngle;
	if (i==0)
		echo0=robot.GetScanDistBack(nbPingByScan-1);
	else
		echo0=robot.GetScanDistFront(i-1);
	endif
	echo1=robot.GetScanDistFront(i);
	echo2=robot.GetScanDistFront(i+1);
	if (echo0>=minDistance && echo1>=minDistance && echo2>=minDistance)
		validMoveList(i+1,1)=2*echo1+echo0+echo2;
		validMoveList(i+1,2)=min([echo0/3,echo1/3,echo2/3]);
		validMoveList(i+1,3)=angleR;
	endif
	if (i==0)
		echo0=robot.GetScanDistFront(nbPingByScan-1);
	else
		echo0=robot.GetScanDistBack(i-1);
	endif
	echo1=robot.GetScanDistBack(i);
	echo2=robot.GetScanDistBack(i+1);
	if (echo0>=minDistance && echo1>=minDistance && echo2>=minDistance)
		validMoveList(i+1,4)=2*echo1+echo0+echo2;
		validMoveList(i+1,5)=min([echo0/3,echo1/3,echo2/3]);
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