 function [apRobot,robot,ShiftNO] = ToolCalibrateCompass(apRobot,robot)
     %{
     use [apRobot,robot,ShiftNO] = ApCalibrateCompass() to start robot real mode
     then use [apRobot,robot,ShiftNO] = ApCalibrateCompass(apRobot,robot) to retry
     %}
    printf(mfilename);
    printf("   ***   ");
    printf(ctime(time()))
    NOReferenceLocation=[184,184,0];
    EchoReferenceF=242;
    EchoReferenceB=93;;
    % currentShiftNorthOrientation
     if (!exist("robot"))  %
    [apRobot,robot] = ApStartRobotRealmode(1,1)
    endif
    robot.SetUdpTraceNO(true);
    printf(mfilename);      
    printf("  check front back distance with ping is consistent with (184,184,0) ***   ");
    printf(ctime(time()))
    robot.RobotAlignServo(90);
    pause(2);
    robot.PingEchoFrontBack();
    pause(5);
    echoF=robot.lastEchoFront;
    echoB=robot.lastEchoBack;
    if (abs((EchoReferenceF-echoF)/EchoReferenceF)>0.05 || abs((EchoReferenceB-echoB)/EchoReferenceB)>0.07)
          printf(mfilename);      
          printf("  check location ping FB not consistent - expected:(%d,%d) - got:(%d,%d) ***   ",EchoReferenceF,EchoReferenceB,echoF,echoB);
          printf(ctime(time()))
    endif  
    [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,NOReferenceLocation);
    NOReference=scanRef(3)
    validate=false;
    compassMode=9;
    NDOFMode=12;
    while (robot.BNOMode!=compassMode)
      printf(".");
      pause(1)
    endwhile
    while (validate==false)
      printf(mfilename);
      printf("   Is robot ready, BNO green LED on & located (%d,%d,%d) ? ",NOReferenceLocation(1),NOReferenceLocation(2),NOReferenceLocation(3));
      validate=yes_or_no("yes or no");
    end
    NO=-1;
    while (NO==-1)
      NO=robot.RefreshNorthOrientation();
      printf(".");
      pause(1);
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