function [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot)
idx=1;
retCode=99;
typeWait=apGet(apRobot,"waitFor");
while (retCode==99)
	source=robot.eventOctave;
	dest=robot.GetEventArduinoDest(typeWait);
	retCode=robot.GetRetcode(typeWait,source,dest);          % wait 
	if (mod(idx,10)==0 || (retCode!=0 && retCode!=99 ))
    printf(mfilename);
		printf(" typeWait:%d source:%d  dest:%d retcode:%d. *** ",typeWait,source,dest,retCode);
		printf(ctime(time()));
	endif
	idx++;
	pause(1)
end
endfunction