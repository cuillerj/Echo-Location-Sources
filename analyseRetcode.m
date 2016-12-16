function [issue,action] = analyseRetcode(robot,retCode,currentAction,callFrom)

if retCode==-1
	printf("timeout action: %d  called from: %d *** ",currentAction,callFrom)
	printf(ctime(time()))
	issue=true;
	action="stop..";
else
	testedValue=robot.moveEnd;
	if (currentAction == testedValue)
		testedValue=robot.moveUnderLimitation;
		if (retCode == testedValue)
				action="noMove."
		endif
		testedValue=robot.moveKoDueToSpeedInconsistancy
		if (retCode == testedValue)
				action="inMove"
		endif
		testedValue=robot.moveKoDueToObstacle
		if (retCode == testedValue)
				action="obstac"
		endif
	endif
	testedValue=robot.pingFBEnd;
	if (currentAction == testedValue)
		if (retCode == 90)
			printf("Simulation function:%d *** ",currentAction)
			printf(ctime(time()))
			retCode=0;    % force RC=0
		endif
	endif
	issue=false;
	action="resume";
endif
return