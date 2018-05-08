  function [result] = TfGetRotatedTensorFlowResult()
  while(exist("rotatedTensorFlow/newResponse.txt")==0)
    pause(1)
  endwhile
  delete("rotatedTensorFlow/newResponse.txt")
  result=csvread ("rotatedTensorFlow/result.csv");
endfunction