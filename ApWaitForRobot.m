function [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn)
if (!exist("debugOn"))  % flat logistic regression is default mode 
       debugOn=true;
endif
idx=1;
retCode=99;
typeWait=apGet(apRobot,"waitFor");
while (retCode==99 && robot.runningStatus >=0 )
	source=robot.eventOctave;
	dest=robot.GetEventArduinoDest(typeWait);
	retCode=robot.GetRetcode(typeWait,source,dest);          % wait 
	if (mod(idx,10)==0 || (retCode!=0 && retCode!=99 ))
    printf(mfilename);
		printf(" typeWait:%d source:%d  dest:%d retcode:%d. *** ",typeWait,source,dest,retCode);
		printf(ctime(time()));
	endif
	idx++;
  printf("+");
	pause(1);
end
printf("\n")
if (robot.runningStatus<0)
  retCode=-99;
 endif
endfunction