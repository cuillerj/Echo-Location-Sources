function [apRobot,robot,result,retCode]=ApScanToTensorFlowFlat (apRobot,robot,plotOn,scanId)
  %{

   %}
 if (!exist("plotOn"))
    plotOn=false;
  endif
 if (!exist("scanId"))
    scanId=0;
 endif
  retCode=9;
  [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,0,scanId);
 if (retCode==0)
    printf(mfilename);
    printf("  scan ended  ***  ")
    printf(ctime(time()))
    robot.SetRunningStatus(0);
    [apRobot,robot,tensorFlowDataIn,newScan] = TfCreateTensorFlowDataIn(apRobot,robot,0,plotOn);
    [result] = TfGetTensorFlowResult();
%    [X,Y,Angle,Cost]=ApAnalyseLastScanRotation(robot,false,nbPred,heading,plotOn);
  else
    if (action=="retry.")  % retry ounce
       retCode=9;
      [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,0,scanId);
      if (retCode==0)
        printf(mfilename);
        printf("  rerty scan ended  ***  ")
        printf(ctime(time()))
        robot.SetRunningStatus(0);
        [apRobot,robot,tensorFlowDataIn,newScan] = TfCreateTensorFlowDataIn(apRobot,robot,0,plotOn);
        [result] = TfGetTensorFlowResult();
      endif
     endif
  endif
endfunction
