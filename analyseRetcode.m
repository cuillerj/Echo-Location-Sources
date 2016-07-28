function [issue,action] = analyseRetcode(retCode,currentAction,callFrom)
if retCode==-1
	printf("timeout action: %d  called from: %d ",currentAction,callFrom)
	printf(ctime(time()))
	issue=true;
	action="stop..";
else
	issue=false;
	action="resume";
endif
return