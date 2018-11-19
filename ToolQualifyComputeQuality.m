function [quality] = ToolQualifyComputeQuality(data)
 % compute score of a prediction versus actual position
 %     result=[result;[scanId,scanRef,posX,posY,posH,posProb,robot.GetScanNOOrientation(),retCode]];
    % NO comparaison
    NOQuality=ToolAngleDiff(data(4),data(size(data,2)-1));
    % location comparaison
    len=(size(data,2)-7)/3;
    posX=data(5:len+4);
    posY=data(len+5:2*len+4);
    prob=data(2*len+6:3*len+5);
    pos=[posX;posY]';
    actual=[ones(len,1)*data(2),ones(len,1)*data(3)];
    delta=sqrt((pos(:,1)-actual(:,1)).^2+(pos(:,2)-actual(:,2)).^2);
    [min,idx]=min(delta);
    foundPosition=0;
    if (min==0)
      printf("found:%d \n",idx)
      foundPosition=idx;
    else
      printf("closest:(%d,%d) distance:%d idx:%d \n",posX(idx),posY(idx),min,idx)
    endif
    quality=[data(1),foundPosition,NOQuality,posX(idx),posY(idx),round(delta(idx))];
endfunction