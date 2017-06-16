function [rotation,distance,possible] = CheckStraightMovePossibility(carto,currentX,currentY,currentHeading,targetX,targetY)
deltaX=targetX-currentX;
deltaY=targetY-currentY;
possible=true;
rotation=0;
distance=0;
if (deltaX!=0)
	deltaHeading=atan(deltaY / deltaX) * 180 / pi();
	rotation=deltaHeading-currentHeading;
else
	if (deltaY>=0)
		rotation=90;
	else
		rotation=-90;
	endif
endif
if (deltaX<0)
	rotation = 180 + rotation;
    if (rotation > 180)
        rotation = rotation - 360;  
	endif
endif
newAngle=mod(rotation+currentHeading,360);
cosA=cos(mod(rotation+currentHeading,360)*pi()/180);
sinA=sin(mod(rotation+currentHeading,360)*pi()/180);
distance = sqrt(deltaX^2 + deltaY^2);
delta=10;
while (delta<distance && possible==true)
	x=floor(currentX+delta*cosA);
	y=floor(currentY+delta*sinA);
	if (QueryCartoAvailability(carto,floor(currentX+delta*cosA),floor(currentY+delta*sinA),newAngle*pi()/180,0)==false)
		possible=false;
	endif
	delta=delta+10;
end