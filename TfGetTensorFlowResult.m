  function [result] = TfGetTensorFlowResult()
   idx=0;
  printf(mfilename);
  printf("  wait for tensorFlow ***  ");
  printf(ctime(time()))
  while(exist("tensorFlow/newResponse.txt")==0)
    pause(1)
    idx++;
    if (mod(idx,10)==1)
      printf("+");
    endif
    pause(1);
  endwhile
  delete("tensorFlow/newResponse.txt")
  pause(2)
  result=csvread ("tensorFlow/result.csv");
endfunction