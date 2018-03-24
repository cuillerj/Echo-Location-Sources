function [param,value,found] = ApeRobotCommonDefine(input)
  % not longuer used
% a remplacer par call [param,value,number] = GetParametersValueByName(robot,input,parametersNameList
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
	"stepSize",
	"nbPulse"};
	
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
	10;
	15;
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