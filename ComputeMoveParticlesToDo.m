function [rotationParticlesToDo,lenParticlesToDo] = ComputeMoveParticlesToDo(currentX,currentY,currentHeading,nextX,nextY,forward,robot,parametersNameList)
deltaX=nextX-currentX;
deltaY=nextY-currentY;
currentHeadingGrad=currentHeading*pi/180;
if ((nextX-currentX)!=0)
%	if((nextY-rotationCenterY)!=0)
		deltaAlpha=atan((nextY-currentY)/(nextX-currentX))*180/pi
%	else
%		deltaAlpha=0
%	endif
else 
	if ((nextY-currentY)>=0)
		deltaAlpha=90;
	else
		deltaAlpha=270;
	endif
endif
rotationParticlesToDo=deltaAlpha-currentHeading;
%[param,value,found] = ApeRobotCommonDefine("minRotToBeDone");
[p1,value,p3]=GetParametersValueByName(robot,"minRotToBeDone",parametersNameList);
if (abs(rotationParticlesToDo)<value)
	if (value-abs(rotationParticlesToDo)<value/2)
		rotationParticlesToDo=value*sign(rotationParticlesToDo);
	else
		rotationParticlesToDo=0;
	endif
endif
if (nextX-currentX<0)
	rotationParticlesToDo=180+rotationParticlesToDo;
endif
if (forward==-1)
%	rotationParticlesToDo=-rotationParticlesToDo;
	rotationParticlesToDo=mod(rotationParticlesToDo+180,360);
endif
rotationParticlesToDo=round(mod(rotationParticlesToDo+360,360));
lenParticlesToDo=round(sqrt((nextX-currentX)^2+(nextY-currentY)^2))*forward;
printf("Compute Move Patricles rotation:%d dist:%d. *** ",rotationParticlesToDo,lenParticlesToDo);
printf(ctime(time()));
endfunction