 function [typeWaitString,typeWaitMapping] = ApTypeWaitData()
  typeWaitString={"robotInfoUpdated";"robotUpdatedEnd";"scanning";"moving";"scanEnd";"moveEnd";"northAlignEnd";"servoAlignEnd";"pingFBEnd";"moveAcrossPassEnded";"requestBNOEnd";"robotNOUpdated";"moveAcrossPassEnded"};
  typeWaitMapping=[[1,1];[8,2];[102,3];[104,4];[103,5];[105,6];[107,7];[108,8];[109,9];[112,10];[118,11];[123,12];[129,13]];
 endfunction