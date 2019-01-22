function [apRobot,robot,result,retCode]=ApScanToTensorFlowRotated (apRobot,robot,plotOn,scanId)
  %{

   %}
 nbMesurementByTrain=apGet(apRobot,"nbMesurementByTrain");
 scanRange=[0: 180/(nbMesurementByTrain-1):360];
 if (!exist("plotOn"))
    plotOn=false;
  endif
 if (!exist("scanId"))
    scanId=0;
 endif
  retCode=9;
  [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,0,scanId);
  location=apGet(apRobot,"location");
  heading=mod(location(3),360);
  headingMatrix=ones(1,size(scanRange,2))*heading;
  [value,step]=min(abs(scanRange-headingMatrix));
 if (retCode==0)
    printf(mfilename);
    printf("  scan ended  ***  ")
    printf(ctime(time()))
    robot.SetRunningStatus(0);
    [apRobot,robot,tensorFlowDataIn,newScan] = TfCreateTensorFlowDataIn(apRobot,robot,step-1,plotOn);
    [result] = TfGetTensorFlowResult();
%    [X,Y,Angle,Cost]=ApAnalyseLastScanRotation(robot,false,nbPred,heading,plotOn);
  else
    if (action=="retry.")  % retry ounce
       retCode=9;
      [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,0,scanId);
      if (retCode==0)
        printf(mfilename);
        printf("  scan ended  ***  ")
        printf(ctime(time()))
        robot.SetRunningStatus(0);
        [apRobot,robot,tensorFlowDataIn,newScan] = TfCreateTensorFlowDataIn(apRobot,robot,rotation,plotOn);
        [result] = TfGetTensorFlowResult();
      endif
     endif
  endif
  
endfunction
