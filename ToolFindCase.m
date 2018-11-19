 function [caseNumber] = ToolFindCase(x,y)
   load ('casesNumber.txt');
   fx=casesNumber(:,1)==x;
   fy=casesNumber(:,2)==y;
   idx=find(fx.*fy,1);
   caseNumber=casesNumber(idx,3);
 endfunction