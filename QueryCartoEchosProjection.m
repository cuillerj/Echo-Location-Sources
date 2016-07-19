function [p0,p1,p2,p3] = QueryCartoEchosProjection(carto,posX,posY)
[a,b]=size(carto);
p0=p1=p2=p3=[0,0];
for x=posX:a
	if(carto(x,posY)==200)
		p0=[x,posY];
		x=a+1;
		break
	endif
	x++;
endfor
for y=posY:b
	if(carto(posX,y)==200)
		p1=[posX,y];
		y=b+1;
		break
	endif
	y++;
endfor
for x=1:posX-1
	if(carto(posX-x,posY)==200)
		p2=[posX-x,posY];
		x=a+1;
		break
	endif
	x++;
endfor
for y=1:posY-1
	if(carto(posX,posY-y)==200)
		p3=[posX,posY-y];
		x=b+1;
		break
	endif
	y++;
endfor

endfunction