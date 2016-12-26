load traceEcho
load "actual.txt"
load carto1img;
load traceDet;
img=carto1img;
trajectories=figure()
title ("trajectories");

hold on;
imshow(img,[])
[a,b]=size(img);
text(a-10,b-100,"actual","color",'r');
text(a-10,b-90,"theoritical","color",'g');
text(a-10,b-80,"robot","color",'b');
text(a-10,b-70,"gyroscope","color",'m');
text(a-10,b-60,"determined","color",'k');
for i=1:size(actual,1)
	plot(actual(i,1),a+1-actual(i,2),'r','marker','o');
end
for i=1:size(traceEcho,1)
	plot(traceEcho(i,3),a+1-traceEcho(i,6),'g','marker','*');
end
for i=1:size(traceEcho,1)
	plot(traceEcho(i,4),a+1-traceEcho(i,7),'b','marker','+');
end
for i=1:size(traceEcho,1)
	plot(traceEcho(i,5),a+1-traceEcho(i,8),'m','marker','s');
end
for i=1:size(traceDet,1)
	plot(traceDet(i,3),a+1-traceDet(i,4),'k','markersize',10);
end
hold off;