function [] = ApStopRobot(apRobot,robot)
    ApStopRobotJava(robot)
    printf(mfilename);
    printf("  end *** ")
    printf(ctime(time()));
    quit
 endfunction