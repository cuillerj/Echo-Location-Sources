 function [apRobot,robot] = ApMaintenance(apRobot,robot)
  printf(" Exit: 0\n")
  printf(" Power on sensors: 1\n")
  printf(" Power off sensors: 2\n")
  printf(" Set UDP trace on: 3\n")
  printf(" Set UDP trace off: 4\n")
  printf(" Set NO trace on: 5\n")
  printf(" Set NO trace off: 6\n")
  printf(" Set file trace on: 7\n")
  printf(" Set file trace off: 8\n")
  printf(" Start tensor flow: 9\n")
  printf(" Stop tensor flow: 10\n")
  printf(" Sleep on: 11\n")
  printf(" Sleep off: 12\n")
  printf(" Request Version: 13\n")
  printf(" Ping  front back: 14\n")
  printf(" Get Subsystem Location: 15\n")
  printf(" GetS ubsytem Registers: 16\n")
  printf(" Stop robot server:17\n")
  printf(" Init robot value:18\n")
  printf(" Lanch batch:19\n")
  printf(" Stop all: 20\n")
  option=input("enter your choice ");
  switch(option)
    case(0)
      return;
    case(1)
      robot.SetPowerEncoder(1);
    case(2)
      robot.SetPowerEncoder(0);
    case(3)
      robot.SetUdpTrace(1);
    case(4)
        robot.SetUdpTrace(0);
     case(5)
      robot.SetUdpTraceNO(1);
    case(6)
        robot.SetUdpTraceNO(0);
    case(7)
      robot.SetTraceFileOn(1);
    case(8)
        robot.SetTraceFileOn(0);
      case(9)
      robot.StartTensorFlowPrediction();
    case(10)
        robot.StopTensorFlow();
    case(11)
      robot.setSleepMode(1);
    case(12)
        robot.setSleepMode(0);
     case(13)
      robot.RequestVersion();
    case(14)
      robot.PingEchoFrontBack();
    case(15)
      robot.GetSubsystemLocation();
     case(16)
      robot.GetSubsytemRegisters();
    case (17)
      robot.StopRobotServer;
    case (18)
      robot.InitRobotValues;
     case (19)       
       if (apGet(apRobot,"realMode"))
        rcBatch=robot.LaunchBatch();        % call java method to start all batchs
      else
        rcBatch=robot.LaunchSimu();        % call java method to start only simulator
        endif
        if (rcBatch!=0)
            printf(mfilename);
            printf(" java batch launch issue:%d - Need to restart Octave *** ",rcBatch);
            printf(ctime(time()));
            return
        endif
     case(20)
      ApStopRobot(apRobot,robot);
  endswitch
  return
 endfunction