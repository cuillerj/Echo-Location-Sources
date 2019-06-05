function [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn)
  if (!exist("debugOn"))  
         debugOn=true;
  endif
  [typeWaitString,typeWaitMapping] = ApTypeWaitData();
   pause(1); % to avoid speedy loop
  %typeWaitString={"robotInfoUpdated";"robotUpdatedEnd";"scanning";"moving";"scanEnd";"moveEnd";"northAlignEnd";"servoAlignEnd";"pingFBEnd";"moveAcrossPassEnded";"requestBNOEnd";"robotNOUpdated";"moveAcrossPassEnded"};
 % typeWaitMapping=[[1,1];[8,2];[102,3];[104,4];[103,5];[105,6];[107,7];[108,8];[109,9];[112,10];[118,11];[123,12];[129,13]];
  idx=1;
  retCode=99;
  typeWait=apGet(apRobot,"waitFor");
    %{
    actions list
    %}
    moveStraight=1;
    rotate=2;
    northAlign=3;
    scan360=4;
    determine=5;
    pingFB=6;
  %while (retCode==99 && robot.runningStatus >=0 )
  while (retCode==99 )
    source=robot.eventOctave;
    dest=robot.GetEventArduinoDest(typeWait);
    retCode=robot.GetRetcode(typeWait,source,dest) ;         % wait 
    if (mod(idx,10)==0 || (retCode!=0 && retCode!=99 ))
      printf(mfilename);
 %     twchar=char(typeWaitString(find(typeWaitMapping(:,1)==typeWait)));
      twchar=ApActionList(apRobot,robot,typeWait);
      printf(" typeWait:(%d:%s) source:%d  dest:%d retcode:%d realMode:%d. *** ",typeWait,twchar,source,dest,retCode,apGet(apRobot,"realMode"));
      printf(ctime(time()));
      if(typeWait==107)
            printf(mfilename);
            printf(" BNO mode:%d  CalStat:%d SysStat:%d SysError:%d *** ",robot.BNOMode,robot.BNOCalStat,robot.BNOSysStat,robot.BNOSysError);
            printf(ctime(time()));
      endif
    endif
    idx++;
    if (mod(idx,10)==1)
      if (typeWait==robot.scanEnd)
        printf("+%d",robot.scanReceiveCount);
      else
         printf("+");
      endif
    endif
    pause(1);
  end
  if (idx>10)
    printf("\n")
  endif
  printf(mfilename);
 % twchar=char(typeWaitString(find(typeWaitMapping(:,1)==typeWait)));
  twchar=ApActionList(apRobot,robot,typeWait);
  printf(" got event:(%d:%s) source:%d  dest:%d retcode=%d:%s*** ",typeWait,twchar,source,dest,retCode,ApRetcodeString(apRobot,robot,typeWait,retCode));
  printf(ctime(time()));
  if (typeWait==robot.northAlignEnd)
      [apRobot,robot,newState,rc] = ApAutomaton(apRobot,robot,[northAlign,retCode],1);
  endif
  if (typeWait==robot.moveEnd && apGet(apRobot,"currentMove")=="rotate")
      [apRobot,robot,newState,rc] = ApAutomaton(apRobot,robot,[rotate,retCode],1);
  endif
  if (typeWait==robot.moveEnd && apGet(apRobot,"currentMove")=="straig")
      [apRobot,robot,newState,rc] = ApAutomaton(apRobot,robot,[moveStraight,retCode],1);
  endif
  if (typeWait==robot.scanEnd)
      [apRobot,robot,newState,rc] = ApAutomaton(apRobot,robot,[scan360,retCode],1);
  endif
  if (retCode==-1) % timeout
   robot.RequestInternalFlags();
   pause(3);
   robot.ResetRobotStatus() % reset robot
   robot.RequestInternalFlags();
    pause(3);
  endif
  pause(1);

endfunction