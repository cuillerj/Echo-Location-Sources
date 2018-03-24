 function [apRobot,robot,ShiftNO] = ApCalibrateCompass(apRobot,robot)
  printf(mfilename);
  printf("   ***   ");
  printf(ctime(time()))
  NOReferenceLocation=[184,184,0];
   if (!exist("robot"))  % flat logistic regression is default mode 
  [apRobot,robot] = ApStartRobotRealmode(1,1)
  endif
  [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,NOReferenceLocation);
  NOReference=scanRef(3)
  validate=false;
  while (validate==false)
    printf(mfilename);
    printf("   Is robot ready, BNO green LED on & located (%d,%d,%d) ? ",NOReferenceLocation(1),NOReferenceLocation(2),NOReferenceLocation(3));
    validate=yes_or_no("yes or no");
  end
  compassMode=9;
  robot.setBNOMode(compassMode);
  pause(5);
  NO=-1;
  while (NO==-1)
    NO=robot.RefreshNorthOrientation();
    printf(".");
    pause(5);
  end
  ShiftNO=NO-NOReference;
  endfunction