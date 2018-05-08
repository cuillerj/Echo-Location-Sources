function [new] = ExtrapolateScan(tensorFlowDataIn,incrementValue)
  # tensorFlowDataIn flat scan vector [front,back ]
  # incrementValue nb of extrolation to do between 2 mesurments
  newFront=[];
  [dl,dc]=size(tensorFlowDataIn);
  for i=1:(dc/2-1)
    distance=(tensorFlowDataIn(:,i+1)-tensorFlowDataIn(:,i))/(incrementValue+1);
    for j=0:(incrementValue)
      newFront=[newFront,[round(tensorFlowDataIn(:,i)+j*distance)]];
    endfor
  endfor
  newBack=[tensorFlowDataIn(:,dc/2)];
  for k=dc/2+1:dc-1
    distance=(tensorFlowDataIn(:,k+1)-tensorFlowDataIn(:,k))/(incrementValue+1);
    for j=0:(incrementValue)
      newBack=[newBack,[round(tensorFlowDataIn(:,k)+j*distance)]];
    endfor
  endfor
  new=[newFront,newBack,[tensorFlowDataIn(:,dc)]];
endfunction