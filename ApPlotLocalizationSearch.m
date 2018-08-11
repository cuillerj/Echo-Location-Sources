function [] = ApPlotLocalizationSearch(apRobot,traceLoc)
load carto1img;
img=carto1img;
trajectories=figure()
title ("trajectories *:actual: x:determined ->:orientation / o: expected   ");
shitfCartoX=apGet(apRobot,"shitfCartoX");
shitfCartoY=apGet(apRobot,"shitfCartoY");
[a,b]=size(img);
hold on;
grid minor on ;
imshow(img,[]);
axis([1,b+20,1,a+20],"on","xy");
#text(a-10,b-320,"actual:* - determined:x");
colorlist=['b';'r';'k';'g';'c';'y';'m';'w'];
set (0, "defaultlinecolor", "black");
traceDet=traceLoc(:,3:5);
actual=traceLoc(:,15:17);
expected=traceLoc(:,9:10);
for i=1:size(actual,1)
	%plot(traceDet(i,3),a+1-traceDet(i,4),'k','markersize',10);
		plot(actual(i,1)+shitfCartoX,actual(i,2)+shitfCartoY,colorlist(mod(i-1,8)+1),'marker','*','markersize',12);
end
for i=1:size(traceDet,1)
	%plot(traceDet(i,3),a+1-traceDet(i,4),'k','markersize',10);
		plot(traceDet(i,1)+shitfCartoX,traceDet(i,2)+shitfCartoY,colorlist(mod(i-1,8)+1),'marker','x','markersize',10);
end
for i=1:size(expected,1)
	%plot(traceDet(i,3),a+1-traceDet(i,4),'k','markersize',10);
		plot(expected(i,1)+shitfCartoX,expected(i,2)+shitfCartoY,colorlist(mod(i-1,8)+1),'marker','o','markersize',10);
end
for i=1:size(actual,1)
  if (actual(i,1)<traceDet(i,1))
    x=actual(i,1):1:traceDet(i,1);
  else
    x=traceDet(i,1):1:actual(i,1);
  endif
  a=((traceDet(i,2)-actual(i,2))/(traceDet(i,1)-actual(i,1)));
  b=actual(i,2)-(actual(i,1)*a);
  plot (x+shitfCartoX,a*x+b+shitfCartoY,colorlist(mod(i-1,8)+1));
end
leg=legend("step 1","step 2","step 3","step 4","step 5","step 6","step 7");
set(leg,'color','none');
set (leg, "fontsize", 15);
legend (leg, "location", "east");
hold off;

for i=1:size(actual,1)
  set (0, "defaultlinecolor", colorlist(mod(i-1,8)+1));
  orig=[actual(i,1)+shitfCartoX,actual(i,2)+shitfCartoY];
  tip=[orig+[15*cos((actual(i,3)/180*pi())),15*sin((actual(i,3)/180*pi()))]];
  arrows = [orig tip];
  drawArrowJC (arrows, 5, 1, 0.5, 1);
end
for i=1:size(traceDet,1)
  set (0, "defaultlinecolor", colorlist(mod(i-1,8)+1));
  orig=[traceDet(i,1)+shitfCartoX,traceDet(i,2)+shitfCartoY];
  tip=[orig+[15*cos((traceDet(i,3)/180*pi())),15*sin((traceDet(i,3)/180*pi()))]];
  arrows = [orig tip];
  drawArrowJC (arrows, 5, 1, 0.5, 1);
end
ToolPrintTraceLoc(traceLoc);
endfunction

