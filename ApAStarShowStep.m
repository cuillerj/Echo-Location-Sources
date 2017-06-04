function [] = ApAStarShowStep(apRobot,step,head) 
%{
 plot the steps of a move
 step is a matrix containinf the [(x,y)] points
 head is the title of the graph
 %}
shitfCartoX=apGet(apRobot,"shitfCartoX");
shitfCartoY=apGet(apRobot,"shitfCartoY");
step;
figure()
title (head);

%axis([1,b,1,a],"on","ij");

load carto1img;
img=carto1img;
[a,b]=size(img);
hold on;
imshow(img,[])
axis([1-shitfCartoX,b,-1-shitfCartoY,a],"on","xy");
for i=1:size(step,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
%	plot(step(i,1),a+1-step(i,2),"r")
	plot(step(i,1)+shitfCartoX,step(i,2)+shitfCartoY,"r:x")
endfor
grid minor;
hold off
%{
load "imageCaseCuisine.jpg"
imshow("imageCaseCuisine.jpg",[])

[a,b]=size(imageCaseCuisine);
for i=1:size(step,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
	plot(step(i,1),a+1-step(i,2),"r")
endfor
hold off
%}
endfunction