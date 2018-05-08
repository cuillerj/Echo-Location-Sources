function [apRobot,robot,result,retCode]=ApScanToTensorFlowFlat (apRobot,robot)
  %{
  robot.Scan360();
  apRobot = setfield(apRobot,"waitFor",robot.scanEnd);   
  typeWaitString={"robotInfoUpdated";"robotUpdatedEnd";"scanning";"moving";"scanEnd";"moveEnd";"northAlignEnd";"servoAlignEnd";"pingFBEnd";"moveAcrossPassEnded";"requestBNOEnd";"robotNOUpdated"};
  typeWaitMapping=[[1,1];[3,2];[102,3];[104,4];[103,5];[105,6];[107,7];[108,8];[109,9];[112,10];[118,11];[123,12]];
  idx=1;
  retCode=99;
  typeWait=apGet(apRobot,"waitFor");
  while (retCode==99 )
    source=robot.eventOctave;
    dest=robot.GetEventArduinoDest(typeWait);
    retCode=robot.GetRetcode(typeWait,source,dest);          % wait 
    if (mod(idx,10)==0 || (retCode!=0 && retCode!=99 ))
      printf(mfilename);
      twchar=char(typeWaitString(find(typeWaitMapping(:,1)==typeWait)));
      printf(" typeWait:(%d:%s) source:%d  dest:%d retcode:%d. *** ",typeWait,twchar,source,dest,retCode);
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
  if (retCode==-99)
        action="stop..";
        return;                 
   endif
   %}
  retCode=9;
  [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,0);
    if (retCode==0)
    printf(mfilename);
    printf("  scan ended  ***  ")
    printf(ctime(time()))
    robot.SetRunningStatus(0);
    [apRobot,robot,tensorFlowDataIn,newScan] = TfCreateTensorFlowDataIn(apRobot,robot,0,1);
%    [X,Y,Angle,Cost]=ApAnalyseLastScanRotation(robot,false,nbPred,heading,plotOn);
  endif

  [result] = TfGetTensorFlowResult()
endfunction