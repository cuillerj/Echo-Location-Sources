function [apRobot,robot] = TestWheelSpeedControl(apRobot,robot,lenToDo,leftRPS,rightRPS);
  %{
  
  RPS en 1/100°

  %}
  
   %robot.leftMinLimit(0)
  robot.setLeftSetpoint(leftRPS);
  pause(1);
  robot.setRightSetpoint(rightRPS);
  pause(1);
  robot.RequestPID();
  pause(1);
  robot.Move(0,lenToDo);
  
 endfunction