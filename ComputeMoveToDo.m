function [rotationToDo,lenToDo] = ComputeMoveToDo(currentX,currentY,currentHeading,nextX,nextY,forward,robot,parametersNameList)
%{
compute the rotation and move to do taking into account physical charateristics
%}
deltaX=nextX-currentX;
deltaY=nextY-currentY;
currentHeadingGrad=currentHeading*pi/180;
[p1,shiftEchoVsRotationCenter,p3]=GetParametersValueByName(robot,"shiftEchoVsRotationCenter",parametersNameList);
shiftEchoVsRotationCenter=shiftEchoVsRotationCenter/10;
rotationCenterX=currentX-shiftEchoVsRotationCenter*cos(currentHeadingGrad);
rotationCenterY=currentY-shiftEchoVsRotationCenter*sin(currentHeadingGrad);
if ((nextX-rotationCenterX)!=0)
%	if((nextY-rotationCenterY)!=0)
		deltaAlpha=atan((nextY-rotationCenterY)/(nextX-rotationCenterX))*180/pi
%	else
%		deltaAlpha=0
%	endif
else 
	if ((nextY-rotationCenterY)>=0)
		deltaAlpha=90;
	else
		deltaAlpha=270;
	endif
endif
rotationToDo=deltaAlpha-currentHeading;
shiftRotationX=shiftEchoVsRotationCenter*cos(deltaAlpha*pi/180)
shiftRotationY=shiftEchoVsRotationCenter*sin(deltaAlpha*pi/180)
%[param,value,found] = ApeRobotCommonDefine("minRotToBeDone");
[p1,value,p3]=GetParametersValueByName(robot,"minRotToBeDone",parametersNameList);
if (abs(rotationToDo)<value)
	if (value-abs(rotationToDo)<value/2)
		rotationToDo=value*sign(rotationToDo);
	else
		rotationToDo=0;
	endif

endif

if (forward==-1)
	rotationToDo=mod(rotationToDo+180,360);
endif
lenToDo=sqrt((nextX-(rotationCenterX+shiftRotationX))^2+(nextY-(rotationCenterY+shiftRotationY))^2)*forward;
printf("Compute Move Robot rotCenterX:%d rotCenterY:%d rotation:%d dist:%d. *** ",rotationCenterX,rotationCenterY,rotationToDo,lenToDo);
printf(ctime(time()));
[rotationToDo,lenToDo,forward] = OptimizeMoveToDo(rotationToDo,lenToDo,forward)
rotationToDo=round(rotationToDo)
lenToDo=round(lenToDo)
printf("optimize Move Robot  rotation:%d dist:%d. *** ",rotationToDo,lenToDo);
printf(ctime(time()));
endfunction