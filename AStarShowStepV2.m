function [] = AStarShowStepV2(head,step1,color1,step2,color2) % plot the steps of a move
figure()
title (head);
hold on;
load carto1img
imshow(carto1img,[])
[a,b]=size(carto1img);
Msize=5;
for i=1:size(step1,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
	plot(step1(i,1),a+1-step1(i,2),"color",color1,"o","markersize",Msize)
	Msize++;
endfor
Msize=5;
for i=1:size(step2,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
	plot(step2(i,1),a+1-step2(i,2),"color",color2,"s","markersize",Msize)
endfor
hold off
endfunction