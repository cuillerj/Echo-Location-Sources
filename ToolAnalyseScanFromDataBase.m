function [apRobot,robot] = ToolAnalyseScanFromDataBase(apRobot,robot,scan)
  %{
  1)
 
    ( 
     for instance SELECT `scanResult`.`idscan`,
      `scanResult`.`time`,
      `scanResult`.`posX`,
      `scanResult`.`posY`,
      `scanResult`.`angle`,
      `scanResult`.`distFront`,
      `scanResult`.`distBack`,
      `scanResult`.`orientation`,
      `scanResult`.`idCarto`,
      `scanResult`.`trainBatch`
      FROM `robot`.`scanResult`
      where trainBatch in(1,10) or (trainBatch=0 and idscan!=0)
      order by time desc
    save fname.csv
  2)
    scan = csvread('fname.csv')
  3)
   [apRobot,robot] = ToolAnalyseScanFromDataBase(apRobot,robot,scan);
  %}
  scan=scan(2:size(scan),:); % suppress fisrt 0 line
  [scanL,scanR]=size(scan);
  if (scanR!=10)
    printf("scan row size must be 10 instead of:%d \n",scanR);
    return
  endif
  nbScan=(scanL)/15;
  if (nbScan!=round(nbScan))
    printf("scan line size must multiple of 15:%d \n",scanL);
    return
  endif
  close all;
  quality=[];
  for i=0:nbScan-1
    mesurment=scan(15*i+1:15*i+15,5:7);
    posX=scan(15*i+1,3);
    posY=scan(15*i+1,4);
    [apRobot,robot] = ToolLoadEchoesInsideApRobot(apRobot,robot,mesurment);
    [apRobot,robot,tensorFlowDataIn,newScan] = TfCreateTensorFlowDataIn(apRobot,robot,0,0, strcat(num2str(i), ' actual:', '(',num2str(posX),' - ',num2str(posY),')'));
    [result] = TfGetTensorFlowResult();
    [result] = ApConvertTfPredictionForParticlesFilter(result);
    [a,b]=max(result(:,3));
    predictX=result(b,1);
    predictY=result(b,2);
    result=[result,sqrt((result(:,1).-posX).^2+(result(:,2).-posY).^2)];
    [c,d]=min(result(:,size(result,2)));
    if c==0
      found=true;
      position=d;
    else
      found=false;
      position=0;
    endif   
    distance=round(sqrt((predictX-posX)^2+(predictY-posY)^2));
    printf("scan: %i actual (%d,%d), predicted (%d,%d) distance:%d cm found: %d  position:%d \n", i,posX,posY,predictX,predictY,distance,found,position)
    quality = [quality;[i,posX,posY,predictX,predictY,distance,found,position]];
  endfor
  dlmwrite("DBScanQualityPredictions.txt",quality,'delimiter',',','-append');
endfunction