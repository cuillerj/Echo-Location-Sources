function [cost] = MotionCost(rotation,distance,currentX,currentY,currentHeading,plotOn)
% tenir compte du nb entier de trous encodeurs dans rotation cost
cartoId=1;
miniRotationCost=500;
supplementaryCost=200;
[angleHole,distancOneHole]=StepEncoderByHole()
miniRotationAbility=2*angleHole
miniMoveAbility=2*distancOneHole
costD=0;
cosH=cos((currentHeading+rotation)*pi/180);
sinH=sin((currentHeading+rotation)*pi/180);
%rotation/angleHole;
%abs(rotation/angleHole-floor(rotation/angleHole));
%abs(rotation/angleHole-ceil(rotation/angleHole));
if rotation!=0
	if abs(rotation)<miniRotationAbility 	;
		cost=361+miniRotationCost;             % to avoid too small rotation
	else
		if(abs(abs(rotation/angleHole)-abs(floor(rotation/angleHole)))>0.02 && abs(abs(rotation/angleHole)-abs(ceil(rotation/angleHole)))>0.02)
			cost=abs(rotation)+miniRotationCost;
		else 
			cost=miniRotationCost;
		endif
	endif
else
cost=0;
endif
%abs(abs(distance/distancOneHole)-abs(floor(distance/distancOneHole)))
%abs(abs(distance/distancOneHole)-abs(ceil(distance/distancOneHole)))
if (distance!=0)
		if(abs(abs(distance/distancOneHole)-abs(floor(distance/distancOneHole)))>0.05 && abs(abs(distance/distancOneHole)-abs(ceil(distance/distancOneHole)))>0.05)
			costD=abs(distance)+supplementaryCost;
		else 
			costD=abs(distance);
		endif
		if abs(distance)<miniMoveAbility	
			costD=2*costD;
		endif
else
endif
dx=10;
dy=10;
cartoWeight=0;
while (sqrt(dx^2+dy^2)<(distance))
	cartoWeight=cartoWeight+QueryCartoWeight(mod(rotation+currentHeading,360),currentX+dx,currentY+dy,cartoId,plotOn);
	dx=dx+10*cosH;
	dy=dy+10*sinH;
	sqrt(dx^2+dy^2)
end
cartoWeight
cost=cost+costD+cartoWeight;
endfunction