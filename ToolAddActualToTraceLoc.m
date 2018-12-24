function [traceLoc] = ToolAddActualToTraceLoc(traceLoc)
      maxRetry=8;  % must be the same as ApEchoLocalisation
      validate=false;
      while (validate==false)
        loopId=input("loopId: ");
        count=input("count: ");
        idx=((loopId-1)*maxRetry+ count);
        traceLoc(idx,:);
        printf("do you confirm this determined location: X=%d Y=%d H=%d ? ",traceLoc(idx,3),traceLoc(idx,4),traceLoc(idx,5));
        validate=yes_or_no;
        validate=false;
        while (validate==false)
          X=input("actual X: ");
          Y=input("actual Y: ");
          H=mod(input("actual H: "),360);
          printf("do you confirm this actual location: X=%d Y=%d H=%d ? ",X,Y,H);
          validate=yes_or_no;
        end
        d1=sqrt((traceLoc(idx,3)-X)^2+(traceLoc(idx,4)-Y)^2);
        d2=sqrt((traceLoc(idx,9)-X)^2+(traceLoc(idx,10)-Y)^2);
        a1=H;
        if (a1>80)
          a1=a1-360;
        endif
        a2=traceLoc(idx,5);
        if (a2>180)
          a2=a2-360;
        endif
        traceLoc(idx,15:20)=[X,Y,H,d1,d2,a1-a2];
      end
      traceLoc(1,19)=NA;
      traceLoc(1,9:11)=NA;
endfunction