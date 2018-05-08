  function [result] = TfGetTensorFlowResult()
  while(exist("tensorFlow/newResponse.txt")==0)
    pause(1)
  endwhile
  delete("tensorFlow/newResponse.txt")
  pause(1)
  result=csvread ("tensorFlow/result.csv");
endfunction