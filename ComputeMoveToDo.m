function [rotationToDo,lenToDo] = ComputeMoveToDo(currentX,currentY,currentHeading,nextX,nextY,forward,robot,parametersNameList)
deltaX=nextX-currentX;
deltaY=nextY-currentY;
currentHeadingGrad=currentHeading*pi/180;
[p1,shiftEchoVsRotationCenter,p3]=GetParametersValueByName(robot,"shiftEchoVsRotationCenter",parametersNameList);
rotationCenterX=currentX-shiftEchoVsRotationCenter*cos(currentHeadingGrad)
rotationCenterY=currentY+shiftEchoVsRotationCenter*sin(currentHeadingGrad)
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
shiftRotationX=-shiftEchoVsRotationCenter*cos(rotationToDo*pi/180)
shiftRotationY=shiftEchoVsRotationCenter*sin(rotationToDo*pi/180)
%[param,value,found] = ApeRobotCommonDefine("minRotToBeDone");
[p1,value,p3]=GetParametersValueByName(robot,"minRotToBeDone",parametersNameList);
if (abs(rotationToDo)<value)
	if (value-abs(rotationToDo)<value/2)
		rotationToDo=value*sign(rotationToDo);
	else
		rotationToDo=0;
	endif
endif
%{
if (nextX-(rotationCenterX+shiftRotationX)<0)
	rotationToDo=180+rotationToDo;
endif

if (rotationToDo>180);
	rotationToDo=rotationToDo-360;
endif
if (rotationToDo<-180);
	rotationToDo=360+rotationToDo;
endif
%{
if (forward==-1)
	rotationToDo=-rotationToDo;
%	rotationToDo=mod(rotationToDo+180,360);
endif
%}
%}
rotationToDo=rotationToDo;
lenToDo=sqrt((nextX-rotationCenterX+shiftRotationX)^2+(nextY-rotationCenterY+shiftRotationY)^2)*forward;
[rotationToDo,lenToDo,forward] = OptimizeMoveToDo(rotationToDo,lenToDo,forward)
endfunction