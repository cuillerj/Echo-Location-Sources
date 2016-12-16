function [p0,p1,p2,p3] = QueryCartoEchosProjection(carto,posX,posY)
%{
This function ... based on the (X,Y) cartography referential
Parameters are:
	robot: the java object that is used to drive the physical robot
	carto: the cartography matrix
	posX: a value containing a position regarding X axis
	posY: a value containing a position regarding Y axis 
Return 4 points (x,y) 
	regarding (posX,posY) location
		po oriented +X 
		p1 oriented +Y
		p2 oriented -X 
		p3 oriented -Y  		
%}
posX=round(posX);
posY=round(posY);
cartoObstacleValue=200;        % the cartography value that determines a "reliable" obstacle (reliable echo quality)
%[a,b]=size(carto);
[nbLine,nbCol]=size(carto);
p0=p1=p2=p3=[0,0];
for x=posX:nbLine
	if(carto(x,posY)==cartoObstacleValue)
		p0=[x,posY];
		x=nbLine+1;
		break
	endif
	x++;
endfor
for y=posY:nbCol
	if(carto(posX,y)==cartoObstacleValue)
		p1=[posX,y];
		y=nbCol+1;
		break
	endif
	y++;
endfor
for x=1:posX-1
	if(carto(posX-x,posY)==cartoObstacleValue)
		p2=[posX-x,posY];
		x=nbLine+1;
		break
	endif
	x++;
endfor
for y=1:posY-1
	if(carto(posX,posY-y)==cartoObstacleValue)
		p3=[posX,posY-y];
		x=nbCol+1;
		break
	endif
	y++;
endfor

endfunction