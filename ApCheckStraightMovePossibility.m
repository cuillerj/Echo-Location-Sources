function [apRobot,rotation,distance,possible] = ApCheckStraightMovePossibility(apRobot,currentL,targetL,plotOn)
%currentL=apGet(apRobot,"location");
if (!exist("plotOn"))  % flat logistic regression is default mode 
       plotOn=false;
endif
deltaX=targetL(1)-currentL(1);
deltaY=targetL(2)-currentL(2);
currentX=currentL(1);
currentY=currentL(2);
currentHeading=currentL(3);
possible=true;
rotation=0;
distance=0;
debugOn=false;
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
	location=[round(currentX+delta*cosA),round(currentY+delta*sinA),round(newAngle*pi()/180)];
	if (ApQueryCartoAvailability(apRobot,location,0,plotOn)(1)==false)
		possible=false;
	endif
	delta=delta+10;
end