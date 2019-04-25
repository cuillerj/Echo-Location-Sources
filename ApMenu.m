 function [apRobot,robot] = ApMenu(apRobot,robot)
  printf(" Exit: 0\n")
  printf(" Start autonomous robot real mode: 1\n")
  printf(" Start autonomous robot simulation mode: 2\n")
  printf(" Init realmode robot: 3\n")
  printf(" Init simulation mode robot: 4\n")
  printf(" Start manual robot mode: 5\n")
  printf(" Start manual simulation mode random location & target: 6\n")
  printf(" Start manual simulation mode: 7\n")
   printf(" Stop all: 8\n")
   printf(" Maintenance: 9\n")
  option=input("enter your choice ");
  switch(option)
    case(0)
      return;
    case(1)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,1,true,1,apRobot,robot);
    case(2)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,0,true,1,apRobot,robot);
    case(3)
      [apRobot,robot] = ApInitApRobot(1,1,apRobot,robot);
    case(4)
      [apRobot,robot] = ApInitApRobot(1,0,apRobot,robot);
    case(5)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,1,false,2,apRobot,robot);
    case(6)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,0,false,3,apRobot,robot);
     case(7)
      [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(1,0,2,3,apRobot,robot);
     case(8)
      ApStopRobot(apRobot,robot);
      case(9)
      [apRobot,robot] = ApMaintenance(apRobot,robot)
  endswitch
  return
 endfunction