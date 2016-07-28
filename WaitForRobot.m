function [retCode] = WaitForRobot(robot,typeWait)
idx=1;
retCode=9;
while (retCode==9)
	source=robot.eventOctave;
	dest=robot.GetEventArduinoDest(typeWait);
	retCode=robot.GetRetcode(typeWait,source,dest);          % wait 
	if (mod(idx,10)==0 || (retCode!=0 && retCode!=9 ))
		printf("typeWait:%d source:%d  dest:%d retcode:%d. ",typeWait,source,dest,retCode)
	endif
	idx++;
	sleep(1)
end
endfunction