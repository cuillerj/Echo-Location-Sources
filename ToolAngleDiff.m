function [delta] = ToolAngleDiff(alpha,beta)
  delta=mod((360-abs(mod(alpha-beta+360,360))),360);
endfunction