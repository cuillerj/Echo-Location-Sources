function [] = AStarShowPath(path)
figure()
hold on;
x=0
y=0;
for i=1:size(path,1)
%	x=x+path(i,1);
%	y=y+path(i,2);
	if (path(i,1)<0 ||path(i,2)<0)
		plot(path(i,3),path(i,4),"r")
	else
		plot(path(i,3),path(i,4),"b")
	endif
endfor
hold off
endfunction