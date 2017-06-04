function [rc] = InitRobotParameters(robot,parametersNameList);
  %{
  this function get some parameters target values and download this value inside the robot using Java driver
  %}
  
if (robot.runningStatus<=0)
	rc=-1
	return
endif
rc=0;
[param,value,number]=GetParametersValueByName(robot,"leftMotorPWM",parametersNameList);
robot.SetPWMMotor(1,value);
pause(1);
[param,value,number]=GetParametersValueByName(robot,"rightMotorPWM",parametersNameList);
robot.SetPWMMotor(0,value);
pause(1);
[param,value,number]=GetParametersValueByName(robot,"leftToRightDynamicAdjustRatio",parametersNameList);
robot.SetMotorsRatio(value);
pause(1);
[param1,value1,number1]=GetParametersValueByName(robot,"leftIncoderHighValue",parametersNameList);
[param2,value2,number2]=GetParametersValueByName(robot,"leftIncoderLowValue",parametersNameList);
robot.SetEncoderThreshold(1,value2, value1);
pause(1);
[param1,value1,number1]=GetParametersValueByName(robot,"rightIncoderHighValue",parametersNameList);
[param2,value2,number2]=GetParametersValueByName(robot,"rightIncoderLowValue",parametersNameList);
robot.SetEncoderThreshold(0,value2, value1);
pause(1);
robot.QueryEncodersValues();
pause(1);
robot.QueryMotorsPWM();
pause(1);
endfunction