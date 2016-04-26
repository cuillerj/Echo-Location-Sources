function [posX,posY,angle] = ComputeTargetLocation(robot)
cartoId=1
issue=false;
while (issue==false)
	posX=eval(input("enter target X position: ","i"));
	posY=eval(input("enter target Y position: ","i"));
	issue=QueryCartoAvailability(posX,posY,cartoId);
end
angle=0;
endfunction