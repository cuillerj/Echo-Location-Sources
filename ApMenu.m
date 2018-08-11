 function [apRobot,robot] = ApMenu()
  printf(" Start autonomous robot real mode: 1\n")
  printf(" Start autonomous robot simulation mode: 2\n")
  printf(" Init realmode robot: 3\n")
  printf(" Init simulation mode robot: 4\n")
  option=input("enter your choice ");
  switch(option)
    case(1)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,1,1,1)
    case(2)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,0,1,1)
    case(3)
      [apRobot,robot] = ApStartRobotRealmode(1,1)
    case(4)
      [apRobot,robot] = ApInitApRobot(1,0)
  endswitch
  return
 endfunction