function [param,value,found] = ApeRobotCommonDefine(input)
defineList= {
	"iLeftWheelDiameter",
	"iRightWheelDiameter",
	"iRobotWidth",
	"iRobotFrontWidth",
	"leftWheelEncoderHoles",
	"rightWheelEncoderHoles",
	"shiftEchoVsRotationCenter",
	"frontLenght",
	"backLenght",
	"securityLenght",
	"minDistToBeDone",
	"minRotToBeDone",
	"stepSize"};
	
defineValue=[
	6.3; 
	6.3; 
	40;         %40
	30;         %33
    8;
    8;
	11; 
	35;          %35
	12;          %12
	20;
	3;
	5;
	10
	];

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