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
  printf(mfilename);
  printf("  delete newresponse.txt ***  ");
  printf(ctime(time()))
  delete("tensorFlow/newResponse.txt")
  pause(2)
  printf(mfilename);
  printf("  read result.csv ***  ");
  printf(ctime(time()))
  result=csvread ("tensorFlow/result.csv");
  printf(mfilename);
 % printf("  delete result.txt ***  ");
  %printf(ctime(time()))
  %delete("tensorFlow/result.csv")
endfunction