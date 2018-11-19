function [position] = ApGetRadomPosition(apRobot)
   scanRefPoints=apGet(apRobot,"scanRefPoints");
   [nbPoints,width]=size(scanRefPoints);
   idx=max(min(round(rand(1)*nbPoints)+1,nbPoints),1);
   position=scanRefPoints(idx,1:3);
endfunction