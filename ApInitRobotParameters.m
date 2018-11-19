function [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);
  %{
  this function get some parameters target values and download this value inside the robot using Java driver
  %}
  
 printf(mfilename); 
if (robot.runningStatus<=0)
	rc=-1
   printf(mfilename); 
  printf(" unable to download parameters inside robot ");
  printf(ctime(time()));
	return
endif
robot.RequestVersion();
pause(2);
printf(" downloading parameters inside robot ");
printf(ctime(time()));
rc=0;
robot.SetPWMMotor(1,apGet(apRobot,"leftMotorPWM"));
pause(2);
robot.SetPWMMotor(0,apGet(apRobot,"rightMotorPWM"));
pause(2);
robot.SetMotorsRatio(apGet(apRobot,"leftToRightDynamicAdjustRatio"));
pause(2);
robot.SetSlowPWMRatio(apGet(apRobot,"SlowPWMRatio"));
pause(2);
robot.SetMotorPulseLenght(apGet(apRobot,"pulseLenght"));
pause(2);
robot.SetEncoderThreshold(1,apGet(apRobot,"leftIncoderLowValue"), apGet(apRobot,"leftIncoderHighValue"));
pause(2);
robot.SetEncoderThreshold(0,apGet(apRobot,"rightIncoderLowValue"), apGet(apRobot,"rightIncoderHighValue"));
pause(2);
%robot.QueryEncodersValues();
pause(1);
robot.QueryMotorsPWM();
pause(2);
endfunction