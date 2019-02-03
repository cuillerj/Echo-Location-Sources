function [delta] = ToolAngleDiffGrad(alpha,beta)
  %delta=mod((2*pi()-abs(mod(alpha-beta+2*pi(),2*pi()))),2*pi());
  %delta=(mod(alpha,2*pi())-pi())-(mod(beta,2*pi())-pi());
  delta=alpha-beta;
  delta=mod((delta+pi()),2*pi())-pi();
endfunction