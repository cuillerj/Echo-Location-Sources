function [delta] = ToolAngleDiff(alpha,beta)
  %delta=mod((360-abs(mod(alpha-beta+360,360))),360);
  %delta=(mod(alpha-180,360)-180)-(mod(beta,360)-180);
  delta=alpha-beta;
  delta=mod((delta+180),360)-180;
endfunction