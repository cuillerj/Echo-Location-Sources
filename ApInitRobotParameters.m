function [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);
  %{
  this function get some parameters target values and download this value inside the robot using Java driver
  %}
  
 printf(mfilename); 
if (robot.runningStatus<=0)
	rc=-1
  printf(" unable to download paramters inside robot ");
  printf(ctime(time()));
	return
endif
printf(" downloading paramters inside robot ");
printf(ctime(time()));
rc=0;
robot.SetPWMMotor(1,apGet(apRobot,"leftMotorPWM"));
pause(1);
robot.SetPWMMotor(0,apGet(apRobot,"rightMotorPWM"));
pause(1);
robot.SetMotorsRatio(apGet(apRobot,"leftToRightDynamicAdjustRatio"));
pause(1);
robot.SetEncoderThreshold(1,apGet(apRobot,"leftIncoderLowValue"), apGet(apRobot,"leftIncoderHighValue"));
pause(1);
robot.SetEncoderThreshold(0,apGet(apRobot,"rightIncoderLowValue"), apGet(apRobot,"rightIncoderHighValue"));
pause(1);
%robot.QueryEncodersValues();
pause(1);
%robot.QueryMotorsPWM();
pause(1);
endfunction