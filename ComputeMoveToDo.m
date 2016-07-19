function [rotationToDo,lenToDo] = ComputeMoveToDo(currentX,currentY,currentOrientation,nextX,nextY)
deltaX=nextX-currentX;
deltaY=nextY-currentY;
deltaAlpha=atan(deltaY/deltaX)*180/pi;
rotationToDo=deltaAlpha-currentOrientation
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
lenToDo=sqrt(deltaX^2+deltaY^2)
endfunction