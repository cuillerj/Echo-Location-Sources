function [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithTensorFlow(apRobot,robot)
% probability defined according to 28 tests
% first is right: =60% (17/28)
% second is right: =18% 5/28
% third is rignt: =4% 1/28
% fourth is right: = 8% 2/28
% fifth is right: =8% 2/28
% not found in first five =4% 1/28
%probRef=[60,18,06,06,06,04];
  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()))
  [apRobot,robot,result,retCode]=ApScanToTensorFlowFlat (apRobot,robot);
  [result] = ApConvertTfPredictionForParticlesFilter(result);
  posX=result(:,1);
  posX=reshape(posX,1,size(posX,1));
  posY=result(:,2);
  posY=reshape(posY,1,size(posY,1));
  posProb=result(:,3);
  posProb=reshape(posProb,1,size(posProb,1));
  posH=zeros(1,size(posX));
endfunction