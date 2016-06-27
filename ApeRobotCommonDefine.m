function [param,value,found] = ApeRobotCommonDefine(input)
defineList= {
	"iLeftWheelDiameter",
	"iRightWheelDiameter",
	"iRobotWidth",
	"iRobotFrontWidth",
	"leftWheelEncoderHoles"
	"rightWheelEncoderHoles"
	"shiftEchoVsRotationCenter",
	"frontLenght",
	"backLenght",
	"securityLenght",
	"minDistToBeDone",
	"minRotToBeDone",
	"scanning",
	"scanEnded",
	"moving",
	"moveEnded",
	"aligning",
	"alignEnded",
	"moveRetcodeEncoderLeftLowLevel",
	"moveRetcodeEncoderRightLowLevel",
	"moveRetcodeEncoderLeftHighLevel",
	"moveRetcodeEncoderRightHighLevel",
	"moveUnderLimitation",
	"moveKoDueToSpeedInconsistancy",
	"moveKoDueToObstacle",
	"diagMotorPbLeft",
	"diagMotorPbRight",
	"diagMotorPbSynchro",
	"diagMotorPbEncoder",
	"diagRobotPause",
	"diagRobotObstacle"};
	
defineValue=[
	6.4; 
	6.4; 
	46.5;         %46.5
	30;         %33
    8;
    8;
	10; 
	35;          %35
	12;          %12
	0;
	3;
	5;
	102; 
	103;
	104;
	105;
	106;
	107;
	1;
	2;
	3;
	4;
	5;
	6;
	7;
	0;
	1;
	2;
	3;
	0;
	1];
%strDefineList=cellstr(defineList);
%sizeof(strDefineList)
loopexit=false;
found=false;
param="";
value=0.;
i=1;
idx=find(strcmp(input,defineList));

if (idx>0)
	param=defineList(idx);
	value=defineValue(idx);
	found=true;
endif

endfunction