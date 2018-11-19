function [qPstat] = ToolPlotScanQualityPredictionQuality()
  load ("zonesXY.txt");
  nbCase=size(zonesXY,1);
  qP=csvread('DBScanQualityPredictions.txt');
  pbPoss=max(qP(:,8));
  for i=1:pbPoss
    qPstat(1,i)=sum(qP1=qP(:,8)==i);
  endfor
  qP0=sum(qP1=qP(:,8)==0);
  nbScan=sum(qPstat(1,:))+qP0
  qPstat(2,:)=qPstat(1,:)./nbScan;
  for i=2:pbPoss
    qPstat(2,i)=qPstat(2,i)+qPstat(2,i-1);
  endfor
  qPstat(3,1)=1/(nbCase);
  for i=2:pbPoss
    qPstat(3,i)=1/(nbCase-i+1)+qPstat(3,i-1);
  endfor
  figure()
  x=[1:1:pbPoss];
  bar(x,qPstat(1,x));
  title('predicted locations found in x position');
  legend ({'number of time'}, "location", "east");
  set (gca, "yminorgrid", "on");
  figure();
  hold on;
  title('prediction compared to random');
  plot (x,qPstat(2,:))
  plot (x,qPstat(3,:))
  legend ({'prediction','random'}, "location", "east");
  grid minor on;
  hold off
 endfunction