function [retcodeString] = ApRetcodeString(apRobot,robot,action,retcode)
  if (retcode==-1)
      retcodeString="timeout"
  else
    retcodeString=  apGet(apRobot,"retcodeList")(action+1,retcode+1);
  endif
  endfunction