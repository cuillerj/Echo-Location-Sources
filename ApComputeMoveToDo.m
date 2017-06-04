function [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward)
%{
compute the rotation and move to do taking into account physical charateristics
%}
nextX=next(1);
nextY=next(2);
location=apGet(apRobot,"location");
currentX=location(1);
currentY=location(2);
currentHeading=location(3);
deltaX=nextX-currentX;
deltaY=nextY-currentY;
currentHeadingGrad=currentHeading*pi/180;
shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter");
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
minRotToBeDone=apGet(apRobot,"minRotToBeDone");
if (abs(rotationToDo)<minRotToBeDone)
	if (minRotToBeDone-abs(rotationToDo)<minRotToBeDone/2)
		rotationToDo=minRotToBeDone*sign(rotationToDo);
	else
		rotationToDo=0;
	endif

endif

if (forward==-1)

	rotationToDo=mod(rotationToDo+180,360);
endif
lenToDo=sqrt((nextX-(rotationCenterX+shiftRotationX))^2+(nextY-(rotationCenterY+shiftRotationY))^2)*forward;
printf(mfilename);
printf(" Compute Move Robot rotCenterX:%d rotCenterY:%d rotation:%d dist:%d. *** ",rotationCenterX,rotationCenterY,rotationToDo,lenToDo);
printf(ctime(time()));
[rotationToDo,lenToDo,forward] = OptimizeMoveToDo(rotationToDo,lenToDo,forward);
rotationToDo=round(rotationToDo);
lenToDo=round(lenToDo);
printf(mfilename);
printf(" optimize Move Robot  rotation:%d dist:%d. *** ",rotationToDo,lenToDo);
printf(ctime(time()));
endfunction