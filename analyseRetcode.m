function [issue,action] = analyseRetcode(robot,retCode,currentAction,callFrom)

if retCode==-1
	printf("timeout action: %d  called from: %d *** ",currentAction,callFrom)
	printf(ctime(time()))
	issue=true;
	action="stop..";
else
	if (currentAction == robot.moveEnd)
		if (retCode == robot.moveUnderLimitation)
				action="noMove."
		endif
		if (retCode == robot.moveKoDueToSpeedInconsistancy)
				action="inMove"
		endif
		if (retCode == robot.moveKoDueToObstacle)
				action="obstac"
		endif
	endif
	if (currentAction == robot.pingFBEnd)
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