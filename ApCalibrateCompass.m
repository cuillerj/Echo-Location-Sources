 function [apRobot,robot,ShiftNO] = ApCalibrateCompass(apRobot,robot)
     %{
     use [apRobot,robot,ShiftNO] = ApCalibrateCompass() to start robot real mode
     then use [apRobot,robot,ShiftNO] = ApCalibrateCompass(apRobot,robot) to retry
     %}
    printf(mfilename);
    printf("   ***   ");
    printf(ctime(time()))
    NOReferenceLocation=[184,184,0];
    % currentShiftNorthOrientation
     if (!exist("robot"))  %
    [apRobot,robot] = ApStartRobotRealmode(1,1)
    endif
    robot.SetUdpTraceNO(true);
    [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,NOReferenceLocation);
    NOReference=scanRef(3)
    validate=false;
    compassMode=9;
    NDOFMode=12;
    while (robot.BNOMode!=compassMode)
      robot.SetUdpTraceNO(1);
      printf(".");
      pause(5)
    endwhile
    while (validate==false)
      printf(mfilename);
      printf("   Is robot ready, BNO green LED on & located (%d,%d,%d) ? ",NOReferenceLocation(1),NOReferenceLocation(2),NOReferenceLocation(3));
      validate=yes_or_no("yes or no");
    end
   # compassMode=9;
   # robot.setBNOMode(compassMode);
  #  pause(5);
    NO=-1;
    while (NO==-1)
      NO=robot.RefreshNorthOrientation();
      printf(".");
      pause(5);
    end
    ShiftNO=NO-NOReference;
    printf(mfilename);
    printf("   Update database with %d? ",ShiftNO);
    validate=yes_or_no("yes or no");
    if (validate)
      robot.UpdateCurrentShiftNorthOrientation(ShiftNO);
    endif
    robot.SetUdpTraceNO(false);
    pause(5);
    robot.setBNOMode(NDOFMode);
    pause(5);
  endfunction