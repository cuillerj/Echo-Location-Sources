function [r1] = tunningTf()
[result] = TfGetTensorFlowResult()
r1=[];
i=2;
 while (i<=size(result,1))
   r1=[r1;[(result(i,:))]];
   i=i+3;
 end
