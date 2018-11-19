%{
 this function launch  an echo scan and submit the scan result to tensorflow for determination and return list of position x y h prob
%}
function [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithTensorFlow(apRobot,robot,plotOn,scanId)
 if (!exist("plotOn"))
    plotOn=false;
  endif
 if (!exist("scanId"))
    scanId=0;
 endif 
  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()))
  [apRobot,robot,result,retCode]=ApScanToTensorFlowFlat (apRobot,robot,plotOn,scanId);
  if (size(result)==0)
    retCode=-1;
    return
  endif
  [result] = ApConvertTfPredictionForParticlesFilter(result);
  posX=result(:,1);
  posX=reshape(posX,1,size(posX,1));
  posY=result(:,2);
  posY=reshape(posY,1,size(posY,1));
  posProb=result(:,3);
  posProb=reshape(posProb,1,size(posProb,1));
  posH=zeros(1,size(posX));
endfunction