function [] = PlotCartography(cartoId)
load carto1
[x,y]=size(carto1);
figure()
hold on;
for i=1:x
	for j=1:y
%		if (carto1(i,j)=0)
%			plot(carto(i),carto1(j),'g');
%		endif
%		if (carto1(i,j)=1)
%			plot(carto1(i),carto1(j),'b');
%		endif
		if (carto1(i,j)>1)
			plot(carto1(i),carto1(j),'r');
		endif
	endfor
endfor
hold off;
endfunction