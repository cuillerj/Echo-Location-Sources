function [detX,detY,detAngle,detProb,predLocMatrix] = DetermineRobotLocation(robot,posX,posY,posAngle,echoX,echoY,echoAngle,echoProb,predLocMatrix,shiftNorthXOrientation)
%shiftNorthXOrientation=290; % shift between north and X direction in degres
northOrientation=robot.GetNorthOrientation;
northAjdustOrientation=mod((shiftNorthXOrientation-northOrientation+360),360);
currentLocProb=robot.GetCurrentLocProb;
%if (robot.GetHardJustReboot==true || currentLocProb==0) %% robot never located since reboot
	deltaAngle=zeros(size(echoAngle,2),1);
	for i=1:size(echoAngle,2)
		deltaAngle(i)=mod(northAjdustOrientation+360-echoAngle(i),360);
	endfor
	for i=1:size(echoAngle,2)
		if (deltaAngle(i) >=330 || deltaAngle(i) <=30)
			detX=echoX(i);
			detY=echoY(i);
			detAngle=echoAngle(i);
			detProb=min(echoProb(i)*2,90);
			return
		else
			[min,indMin]=min(deltaAngle);
			[max,indMax]=max(deltaAngle);
			if ((360-max)>min)
				detX=echoX(indMin);
				detY=echoY(indMin);
				detAngle=echoAngle(indMin);
				detProb=echoProb(indMin);
			else
				detX=echoX(indMax);
				detY=echoY(indMax);
				detAngle=echoAngle(indMax);
				detProb=echoProb(indMax);
			endif
			if detProb>30
				detProb=30;
			endif
		return
		endif
	endfor	
		robot.SetCurrentLocProb(detProb);
if ( currentLocProb==0) %% robot never located since reboot
	return
else
	return
endif

endfunction