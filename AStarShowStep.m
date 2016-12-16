function [] = AStarShowStep(step,head) % plot the steps of a move
step;
figure()
title (head);
hold on;
load carto1img
imshow(carto1img,[])
[a,b]=size(carto1img);
for i=1:size(step,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
	plot(step(i,1),a+1-step(i,2),"r")
endfor
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