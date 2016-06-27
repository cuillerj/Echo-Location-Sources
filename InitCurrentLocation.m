function [posX,posY,heading,prob] = InitCurrentLocation(robot)
cartoId=1
issue=false;
while (issue==false)
	posX=eval(input("enter current X position: ","i"));
	posY=eval(input("enter current Y position: ","i"));
	heading=eval(input("enter current orientation: ","i"));
	issue=QueryCartoAvailability(posX,posY,heading,cartoId,1);
end
prob=eval(input("enter current probability: ","i"));
endfunction