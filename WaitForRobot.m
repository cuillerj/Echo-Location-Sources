function [retCode] = WaitForRobot(robot,typeWait)
idx=1;
retCode=9;
while (retCode==9)
	retCode=robot.GetRetcode(typeWait,1,2);          % wait 
	if (mod(idx,10)==0)
		printf("robot retcode: %d. \n",retCode);
	endif
	idx++;
	sleep(1)
end
endfunction