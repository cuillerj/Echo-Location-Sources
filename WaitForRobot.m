function [retCode] = WaitForRobot(robot,typeWait)
idx=1;
retCode=99;
while (retCode==99)
	source=robot.eventOctave;
	dest=robot.GetEventArduinoDest(typeWait);
	retCode=robot.GetRetcode(typeWait,source,dest);          % wait 
	if (mod(idx,10)==0 || (retCode!=0 && retCode!=99 ))
		printf("typeWait:%d source:%d  dest:%d retcode:%d. *** ",typeWait,source,dest,retCode);
		printf(ctime(time()));
	endif
	idx++;
	pause(1)
end
endfunction