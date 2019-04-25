function [] = ApStopRobotJava(robot)
   printf(mfilename);
    printf(" Stop Robot Java *** ")
    printf(ctime(time()));
    pause(2);
  robot.StopRobotServer();
     printf(mfilename);
    printf("  *** ")
    printf(ctime(time()));
 endfunction