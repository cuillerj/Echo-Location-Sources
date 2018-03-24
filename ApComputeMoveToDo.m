function [apRobot,robot,rotationToDo,lenToDo] = ApComputeMoveToDo(apRobot,robot,next,forward)
%{
compute the rotation and move to do taking into account physical charateristics
forward 0 means no direction constraint
forward 1 means must move forward
forward -1 means must move backward
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
if ((nextX-rotationCenterX)>0)
%	if((nextY-rotationCenterY)!=0)
		deltaAlpha=atan((nextY-rotationCenterY)/(nextX-rotationCenterX))*180/pi;
    if ((nextX-rotationCenterX)<0)
      forward=-forward;
    endif
%	else
%		deltaAlpha=0
%	endif
elseif ((nextX-rotationCenterX)<0)

		deltaAlpha=180+atan((nextY-rotationCenterY)/(nextX-rotationCenterX))*180/pi;
    if ((nextX-rotationCenterX)<0)

    endif
else 
	if ((nextY-rotationCenterY)>=0)
		deltaAlpha=90;
	else
		deltaAlpha=270;
	endif
endif
rotationToDo=deltaAlpha-currentHeading;
shiftRotationX=shiftEchoVsRotationCenter*cos(deltaAlpha*pi/180);
shiftRotationY=shiftEchoVsRotationCenter*sin(deltaAlpha*pi/180);
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
newX=rotationCenterX+shiftRotationX;
newY=rotationCenterY+shiftRotationY;

lenToDo=sqrt(((nextX-rotationCenterX)^2+(nextY-rotationCenterY)^2))-shiftEchoVsRotationCenter;
printf(mfilename);
printf(" Computed Move Robot rotCenterX:%d rotCenterY:%d rotation:%d dist:%d. *** ",rotationCenterX,rotationCenterY,rotationToDo,lenToDo);
printf(ctime(time()));
if forward==0     % no direction constraint
  forward=1;
  while (abs(rotationToDo)>90)
    [rotationToDo,lenToDo,forward] = ApOptimizeMoveToDo(rotationToDo,lenToDo,forward,shiftEchoVsRotationCenter);
  end
 else
  if (rotationToDo>180)
    rotationToDo=rotationToDo-360;
  endif
  if (rotationToDo<-180)
     rotationToDo=rotationToDo+360;
  endif
endif
rotationToDo=round(rotationToDo);
lenToDo=round(lenToDo)*forward;
printf(mfilename);
printf(" optimized Move Robot  rotation:%d dist:%d. *** ",rotationToDo,lenToDo);
printf(ctime(time()));
endfunction