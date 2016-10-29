function [rotationToDo,lenToDo] = ComputeMoveToDo(currentX,currentY,currentHeading,nextX,nextY,forward)
deltaX=nextX-currentX;
deltaY=nextY-currentY;
if (deltaX!=0)
	deltaAlpha=atan(deltaY/deltaX)*180/pi;
else 
	if (deltaY>=0)
		deltaAlpha=90;
	else
		deltaAlpha=270;
	endif
endif
rotationToDo=deltaAlpha-currentHeading;
[param,value,found] = ApeRobotCommonDefine("minRotToBeDone");
if (abs(rotationToDo)<value)
	rotationToDo=0;
endif
if (deltaX<0)
	rotationToDo=180+rotationToDo;
endif
if (rotationToDo>180);
	rotationToDo=rotationToDo-360;
endif
if (rotationToDo<-180);
	rotationToDo=360+rotationToDo;
endif
if (forward==1)
	rotationToDo=mod(rotationToDo+360,360);
else
	rotationToDo=mod(rotationToDo+180,360);
endif
lenToDo=sqrt(deltaX^2+deltaY^2)*forward;
endfunction