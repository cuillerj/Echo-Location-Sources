function [delta] = ToolAngleDiffGrad(alpha,beta)
  delta=mod((2*pi()-abs(mod(alpha-beta+2*pi(),2*pi()))),2*pi());
endfunction