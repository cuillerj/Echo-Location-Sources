function [apRobot,robot,issue,action] = ApAnalyseRetcode(apRobot,robot,retCode)
currentAction=apGet(apRobot,"waitFor");
callFrom=apGet(apRobot,"callFrom");	
issue=false;
if (retCode==-99)
      printf(mfilename);
			printf("Robot no longer communicate *** ");
			printf(ctime(time()));
      issue=true;
      action="stop..";
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
			printf("Simulation function:%d *** ",currentAction);
			printf(ctime(time()));
			retCode=0;    % force RC=0
		endif
	endif

	action="resume";
endif
return