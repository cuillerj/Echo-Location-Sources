function [apRobot,robot,obstacleHeading,IRMap] = ApCheckIRRobot(apRobot,robot)
    robot.obstacleHeading=999;
    while(robot.obstacleHeading==999)
      robot.GetIRSensors();
      pause(2);
      obstacleHeading=robot.obstacleHeading;
      IRMap=robot.IRMap;
    endwhile
    IRMap=bitget (IRMap, 6:-1:1);
    printf(mfilename); 
    printf(" IR obstacle detection heading:%d map:",obstacleHeading);
    printf("%d",IRMap);
    printf("   ***");
    printf(ctime(time()));
 endfunction