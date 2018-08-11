function [] = ApPlotTrajectory(traceDet,actual)
load carto1img;
img=carto1img;
trajectories=figure()
title ("trajectories colorlist=['b';'r';'k';'g';'w';'y';'m';'c']");
[a,b]=size(img);
hold on;
grid minor on ;
imshow(img,[]);
axis([1,b+20,1,a+20],"on","xy");
text(a-10,b-320,"actual:* - determined:x");
colorlist=['b';'r';'k';'g';'w';'y';'m';'c'];
for i=1:size(actual,1)
	%plot(actual(i,1),a+1-actual(i,2),'r','marker','o');
	plot(actual(i,1),actual(i,2)+50,colorlist(mod(i-1,8)+1),'marker','*','markersize',15);
end
for i=1:size(traceDet,1)
	%plot(traceDet(i,3),a+1-traceDet(i,4),'k','markersize',10);
		plot(traceDet(i,1),traceDet(i,2)+50,colorlist(mod(i-1,8)+1),'marker','x','markersize',10);
end

hold off;

