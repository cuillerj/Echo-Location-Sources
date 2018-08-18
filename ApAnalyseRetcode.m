function [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode)
currentAction=apGet(apRobot,"waitFor");
callFrom=apGet(apRobot,"callFrom");	
issue=false;
action="resume";
if (retCode==-99 && !robot.simulation)
      printf(mfilename);
			printf("Robot no longer communicate *** ");
			printf(ctime(time()));
      issue=true;
      action="stop..";
      return;
 endif
if retCode==-1
	printf("timeout action: %d  called from: %d *** ",currentAction,callFrom);
	printf(ctime(time()));
	%issue=true;
  if (robot.runningStatus==-1)         % robot no longer communicate
    	issue=true;
      action="stop..";
  else
	  action="retry.";
  endif
else
	testedValue=robot.moveEnd;
	if (currentAction == testedValue)
		testedValue=robot.moveUnderLimitation;
		if (retCode == testedValue)
				action="noMove";
		endif
		testedValue=robot.moveKoDueToWheelStopped;
		if (retCode == testedValue)
				action="inMove";
		endif
		testedValue=robot.moveKoDueToObstacle;
		if (retCode == testedValue)
				action="obstac";
		endif
	endif
	testedValue=robot.pingFBEnd;
	if (currentAction == testedValue)
		if (retCode == 90)
      printf(mfilename);
			printf(" Simulation function:%d *** ",currentAction);
			printf(ctime(time()));
			retCode=0;    % force RC=0
      return;
		endif
	endif
 	testedValue=robot.scanEnd;
	if (currentAction == testedValue)
    printf(mfilename);
		printf(" scanReceiveCount:%d *** ",robot.scanReceiveCount);
		printf(ctime(time()));
		if (robot.scanReceiveCount >13)
      printf(mfilename);
			printf(" action retry *** ");
			printf(ctime(time()));
			action="retry.";
    else
    	issue=true;
      printf(mfilename);
			printf(" action stop *** ");
			printf(ctime(time()));     
      action="stop..";    
      return;
		endif
	endif
	action="resume";
endif
return