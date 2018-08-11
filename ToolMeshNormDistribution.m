function [] = ToolMeshNormDistribution(determined,probability,sigma)
  #sigma=40;
  #determined=[[117,50];[184,117];[184,50]];
  #probability=[0.623;0.212;0.0552]
  figure()
  [l,c]=size(determined);
  tx = ty = linspace (-200, 400, 100)';
  [xx, yy] = meshgrid (tx, ty);
  distance=[];
  r1=sqrt ((determined(1,1)-xx) .^ 2 + (determined(1,2)-yy) .^ 2);
  r2=sqrt ((determined(2,1)-xx) .^ 2 + (determined(2,2)-yy) .^ 2);
  r3=sqrt ((determined(3,1)-xx) .^ 2 + (determined(3,2)-yy) .^ 2);
  #distance = [distance;[sqrt ((determined(i,1)-xx) .^ 2 + (determined(i,2)-yy) .^ 2)]];

  tz = normpdf(r1,0,sigma)*probability(1)+normpdf(r2,0,sigma)*probability(2)+normpdf(r3,0,sigma)*probability(3);
  mesh (tx, ty, tz);
  hold on
  plot(determined(1,1),determined(1,2))
  #tz = normpdf(r,0,40)*0.212;
 # mesh (tx, ty, tz);
endfunction