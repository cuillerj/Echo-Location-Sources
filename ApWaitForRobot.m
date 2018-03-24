function [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn)
  if (!exist("debugOn"))  % flat logistic regression is default mode 
         debugOn=true;
  endif
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
    retCode=robot.GetRetcode(typeWait,source,dest);          % wait 
    if (mod(idx,10)==0 || (retCode!=0 && retCode!=99 ))
      printf(mfilename);
      printf(" typeWait:%d source:%d  dest:%d retcode:%d. *** ",typeWait,source,dest,retCode);
      printf(ctime(time()));
    endif
    idx++;
    if (mod(idx,5)==1)
      printf("+");
    endif
    pause(1);
  end
  if (idx>5)
    printf("\n")
  endif
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

  pause(1);

endfunction