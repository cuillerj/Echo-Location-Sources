function [rc] = InitRobotParameters(robot,parametersNameList);
if (robot.runningStatus<=0)
	rc=-1
	return
endif
rc=0;
[param,value,number]=GetParametersValueByName(robot,"leftMotorPWM",parametersNameList);
robot.SetPWMMotor(1,value);
sleep(1);
[param,value,number]=GetParametersValueByName(robot,"rightMotorPWM",parametersNameList);
robot.SetPWMMotor(0,value);
sleep(1);
[param,value,number]=GetParametersValueByName(robot,"leftToRightDynamicAdjustRatio",parametersNameList);
robot.SetMotorsRatio(value);
sleep(1);
[param1,value1,number1]=GetParametersValueByName(robot,"leftIncoderHighValue",parametersNameList);
[param2,value2,number2]=GetParametersValueByName(robot,"leftIncoderLowValue",parametersNameList);
robot.SetEncoderThreshold(1,value2, value1);
sleep(1);
[param1,value1,number1]=GetParametersValueByName(robot,"rightIncoderHighValue",parametersNameList);
[param2,value2,number2]=GetParametersValueByName(robot,"rightIncoderLowValue",parametersNameList);
robot.SetEncoderThreshold(0,value2, value1);
sleep(1);
robot.QueryEncodersValues();
sleep(1);
robot.QueryMotorsPWM();
sleep(1);
endfunction