function [posX,posY,angle] = ComputeTargetLocation(carto,robot)
cartoId=1
issue=false;
while (issue==false)
	posX=eval(input("enter target X position: ","i"));
	posY=eval(input("enter target Y position: ","i"));
	issue=QueryCartoAvailability(carto,posX,posY,0,1);
end
angle=0;
endfunction