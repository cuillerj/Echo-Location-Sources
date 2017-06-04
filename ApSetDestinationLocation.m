function [apRobot] = ApSetDestinationLocation(apRobot,inputMode);
  defaultDestLocation=[585,135,0];
  destLocation=defaultDestLocation;
  if (inputMode==0)
    return;
  elseif (inputMode==1)
    validate=false;
    while (validate==false)
      destLocation(1)=input("enter destination X: ");
      destLocation(2)=input("enter destination Y: ");
      destLocation(3)=input("enter destination heading: ");
      printf("do you confirm this destination: X=%d Y=%d H=%d ? ",destLocation(1),destLocation(2),destLocation(3));
      validate=yes_or_no("yes or no");
      [available,retCode] = ApQueryCartoAvailability(apRobot,destLocation,0,1);
      if(!available)
         printf("destination not available: X=%d Y=%d H=%d ? ",destLocation(1),destLocation(2),destLocation(3));
         validate=false;
      endif
    end
    apRobot = setfield(apRobot,"destLocation",destLocation);
    apRobot = setfield(apRobot,"newTarget",true);
    return
  else
    printf("input mode not currently supported")
  endif