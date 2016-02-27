function [carto] = MapContent(posX,posY,cartoID)
load "cartoCuisine.txt";
x=floor(posX/10)

if (posX/10-x>.5)
	x=x+1;
endif

y=floor(posY/10)
if (posY/10-y>.5)
	y=y+1;
endif
carto=cartoCuisine(y,x)
endfunction