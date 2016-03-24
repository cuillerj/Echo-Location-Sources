function [] = AStarShowStep(step) % plot the steps of a move
step;
figure()
hold on;
for i=1:size(step,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
	plot(step(i,1),step(i,2),"r")
endfor
hold off
endfunction