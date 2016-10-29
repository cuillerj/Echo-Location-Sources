function [posX,posY,posAngle,posProb,retCode] = EchoLocalizeRobotWithRotation(robot,nbPred,orientation)
% probability defined according to 28 tests
% first is right: =60% (17/28)
% second is right: =18% 5/28
% third is rignt: =4% 1/28
% fourth is right: = 8% 2/28
% fifth is right: =8% 2/28
% not found in first five =4% 1/28
heading=orientation
%probRef=[60,18,06,06,06,04];
probRef=[30,25,20,15,10,5];
retCode=9;
robot.Scan360();
posProb=0;
posX=0;
posY=0;
posAngle=0;
idx=1;
callFrom=10;          % to identify the octave function
WaitFor=robot.scanEnd;
retCode=WaitForRobot(robot,WaitFor); 	% wait for up to date
	if (retCode!=0)
		[issue,action]=analyseRetcode(retCode,WaitFor,callFrom)
		if action=="stop"
			break
		endif
		if action=="break"
			break
		endif
		if action=="resume"
			break
		endif
	endif
	%{
while (retCode==9)
	retCode=robot.GetRetcode(2,1,2);
	if (mod(idx,10)==0)
		printf("robot retcode: %d. ",retCode);
		printf(ctime(time()))
		idx++;
	endif
	sleep(1);
end
%}
if (retCode==0)
	printf("scan ended\n")
	robot.SetRunningStatus(0);
	[X,Y,Angle,Cost]=analyseLastScanRotation(robot,false,nbPred,heading);
	posX=X;
	posY=Y;
	posAngle=Angle;
	for i=1:size(X,2);
	%	posProb(i)=max(floor((probRef-probCum)/3*2),1);
		posProb(i)=probRef(i);
	%	probCum=probCum+posProb(i);
	endfor
endif
%probRef=80;
%probCum=0;
endfunction