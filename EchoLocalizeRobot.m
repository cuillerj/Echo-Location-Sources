function [posX,posY,posAngle,posProb] = EchoLocalizeRobot(robot,nbLocPossibility,nbPred)
% probability defined according to 28 tests
% first is right: =60% (17/28)
% second is right: =18% 5/28
% third is rignt: =4% 1/28
% fourth is right: = 8% 2/28
% fifth is right: =8% 2/28
% not found in first five =4% 1/28
probRef=[60,18,06,06,06,04];
robot.Scan360();
posProb=0;
posX=0;
posY=0;
posAngle=0;
j=0;
while (j<90 )
	sleep(1);
	if (robot.GetRunningStatus()==1)
		printf("scan running\n")
		while (robot.GetRunningStatus()!=2)
			sleep(1);
		end
	endif
	if (robot.GetRunningStatus()==2)
		j=999;
	endif

	j=j+1;
end

if (robot.GetRunningStatus()==2)
		printf("scan ended\n")
	robot.SetRunningStatus(0);
	[X,Y,Angle,Cost]=analyseLastScan(robot,false,nbPred);
	posX=X;
	posY=Y;
	posAngle=Angle;
endif
%probRef=80;
%probCum=0;
for i=1:size(X,2);
%	posProb(i)=max(floor((probRef-probCum)/3*2),1);
	posProb(i)=probRef(i);
%	probCum=probCum+posProb(i);
endfor
endfunction