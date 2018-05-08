function [] = TfPlotRawEchoScan(scanResult)
  [a,b]=size(scanResult);
  str=strcat('echo scan:',mat2str(scanResult(1,7)));
  figure();
  title (str);
  hold on;
  axis("square")
  lim=getSensorDistanceLimit();
  xlim([-lim,lim]);
  ylim([-lim,lim]);
  for i=1:a
    angleRadian=(scanResult(i,4)*pi()/180)-pi()/2;
    x=scanResult(i,5)*cos(angleRadian);
    y=scanResult(i,5)*sin(angleRadian);
    plot(x,y,'*')
    x=scanResult(i,6)*cos(pi()+angleRadian);
    y=scanResult(i,6)*sin(pi()+angleRadian);
    plot(x,y,'*')
  endfor
  hold off
endfunction