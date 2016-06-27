function [angleOneHole,distanceOneHole] = StepEncoderByHole()
% tenir compte du nb entier de trous encodeurs dans rotation cost
	[Param,Value,Found] = ApeRobotCommonDefine("iRobotWidth"); 
	if (Found==true)
		RobotWidth = Value;
	endif
	[Param,Value,Found] = ApeRobotCommonDefine("iLeftWheelDiameter"); 
	if (Found==true)
		WheelDiameter = Value;
	endif
	[Param,Value,Found] = ApeRobotCommonDefine("leftWheelEncoderHoles"); 
	if (Found==true)
		WheelEncoderHoles = Value;
	endif
	angleOneHole=(((pi*WheelDiameter)/WheelEncoderHoles)/(pi*RobotWidth))*360;
	distanceOneHole=((pi*WheelDiameter)/WheelEncoderHoles);
endfunction