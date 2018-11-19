   %{
   this function concatains by locations the raw tensor flow predictions 
   %}
 function [outResult] = ApConvertTfPredictionForParticlesFilter(inResult)
   % nbOfPredictionsByRequest must be consistent with  tensor flow parameter numberOfReturnedValues

  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()));
   nbOfPredictionsByRequest=15;
   probabilityRatio=0.9; % ratio used to mitigate scan shifted +-13° VS 0 oriented
   % lowProbabily=0.5;
   r1=inResult(2:nbOfPredictionsByRequest+1,:);
   r2=inResult(nbOfPredictionsByRequest+2:2*nbOfPredictionsByRequest+1,:);
   r3=inResult(2*nbOfPredictionsByRequest+2:3*nbOfPredictionsByRequest+1,:);
   r2(:,5)=r2(:,5)*probabilityRatio;
   r3(:,5)=r3(:,5)*probabilityRatio;
   r=[r1;r2;r3];
   out=[];
   i=1;
   while i<=size(r,1)
     idxX=r(:,2)==r(i,2);
     idxY=r(:,3)==r(i,3);
     idx=idxX.*idxY;
     probVect=idx.*r(:,5);
    #out=[out;[r(i,2),r(i,3),prod(probVect.*(probVect==0))]];
    out=[out;[r(i,2),r(i,3),sum(probVect)]];
     negidx=idx==0;
     r=negidx.*r;
     i=i+1;
   endwhile

   outResult=[];
   for i=1:size(out)
     if (out(i,3)!=0)
       outResult=[outResult;[out(i,:)]];
     endif
   endfor
   outResult(:,3)=outResult(:,3)/sum(outResult(:,3));
   outResult=sortrows(outResult,3);
   outResult=flipud(outResult);
 endfunction