function [xOut,yOut,hOut,distance] = LookForClosestAvailability(carto,xIn,yIn,heading,debugOn)
% to be developped !!
% heading in grad
% take into account the robot physical characteristics (by checking 10 points) to determine if it can be at a place
%{  
	8___6
    |   |4__________3
	|               |
   9| B X   F       |0
	|    ___________|
	|___|2          1
	7   5
  
	X is the (x,y) reference matching with the echo location
	F distance between x and 0 is the frontLenght
	B distance between x and 9 is the backLenght
	iRobotWidth is the distance between 7 and 8
	iRobotFrontWidth is the distance between 1 and 3
	cosH = cos(heading)
	sinH = sin(heading)
	(x0,y0) = (x + F*cosH , y + F*sinH)
	(x1,y1) = (x + F*cosH + (iRobotFrontWidth/2)*sinH , y + F*sinH - (iRobotFrontWidth/2)*cosH)
	(x3,y3) = (x + F*cosH - (iRobotFrontWidth/2)*sinH , y + F*sinH + (iRobotFrontWidth/2)*cosH)
	(x2,y2) = (x + (iRobotFrontWidth/2)*cosH , y - (iRobotFrontWidth/2)*sinH)
	(x4,y4) = (x - (iRobotFrontWidth/2)*cosH , y + (iRobotFrontWidth/2)*sinH)
	(x5,y5) = (x + (iRobotWidth/2)*sinH , y - (iRobotWidth/2)*cosH)
	(x6,y6) = (x - (iRobotWidth/2)*sinH , y + (iRobotWidth/2)*cosH)
	(x9,y9) = (x - B*cosH , y - B*sinH)
	(x7,y7) = (x - B*cosH + (iRobotWidth/2)*sinH , y - B*sinH - (iRobotWidth/2)*cosH)
	(x8,y8) = (x - B*cosH - (iRobotWidth/2)*sinH , y - B*sins=H + (iRobotWidth/2)*cosH)
		
%}
cartoMaxAvailableValue=20;
[Param,Value,Found] = ApeRobotCommonDefine("iRobotWidth"); 
cosH=cos(heading);
sinH=sin(heading);
xIn=floor(xIn);
yIn=floor(yIn);
if (Found==true)
	RobotBackWidth = Value/2;
endif
[Param,Value,Found] = ApeRobotCommonDefine("stepSize"); 
if (Found==true && Value>1)
	gapSize = Value-1;          % stepSize minus 1 to be sure to detect all obstacles
endif
[Param,Value,Found] = ApeRobotCommonDefine("iRobotFrontWidth"); 
if (Found==true)
	RobotFrontWidth = Value/2;
endif
[Param,Value,Found] = ApeRobotCommonDefine("frontLenght"); 
if (Found==true)
	frontLenght = Value;
endif
[Param,Value,Found] = ApeRobotCommonDefine("backLenght"); 
if (Found==true)
	backLenght= Value;
endif
%{
if (cartoId==1)
	load carto;
else
	load carto0;
endif
%}
[cartoX,cartoY]=size(carto);
available=true;
x=xIn;
y=yIn;
if (x <= 0 || x > cartoX)
	available=false;
	printf("carto not available (1) X:%d Y:%d orientation deg: %f . ",x,y,heading*180/pi())
	printf(ctime(time()))
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	printf("carto not available (1) X:%d Y:%d orientation deg: %f . ",x,y,heading*180/pi())
	printf(ctime(time()))
	return
endif

if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
	printf("carto not available (1) X:%d Y:%d orientation deg: %f . ",x,y,heading*180/pi())
	printf(ctime(time()))
	endif
	return
endif
% check availability between 0 and x
for i=0:floor(frontLenght/gapSize)
	deltaL=frontLenght-gapSize*i;
	x=floor(xIn+deltaL*cosH);
	y=floor(yIn+deltaL*sinH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available center(0-x) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available center (0-x) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available center(0-x) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 9 and x
for i=0:floor(backLenght/gapSize)
	deltaL=backLenght-gapSize*i;
	x=floor(xIn-deltaL*cosH);
	y=floor(yIn-deltaL*sinH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available back(9-x) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available back (9-x) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available back(9-x) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 0 and 1
for i=0:floor(RobotFrontWidth/gapSize)
	deltaL=RobotFrontWidth-gapSize*i;
	x=floor(xIn+frontLenght*cosH+deltaL*sinH);
	y=floor(yIn+frontLenght*sinH-deltaL*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available front(0-1) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available front (0-1) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available front(0-1) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 0 and 3
for i=0:floor(RobotFrontWidth/gapSize)
	deltaL=RobotFrontWidth-gapSize*i;
	x=floor(xIn+frontLenght*cosH-deltaL*sinH);
	y=floor(yIn+frontLenght*sinH+deltaL*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available front(0-3) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available front(0-3) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available front(0-3) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 1 and 2
for i=0:floor(frontLenght/gapSize)
	deltaL=frontLenght-gapSize*i;
	x=floor(xIn+deltaL*cosH+RobotFrontWidth*sinH);
	y=floor(yIn+deltaL*sinH-RobotFrontWidth*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available right side (1-2) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available right side (1-2) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available right side(1-2) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 3 and 4
for i=0:floor(frontLenght/gapSize)
	deltaL=frontLenght-gapSize*i;
	x=floor(xIn+deltaL*cosH-RobotFrontWidth*sinH);
	y=floor(yIn+deltaL*sinH+RobotFrontWidth*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available left side(3-4) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available left side (3-4) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available left side(3-4) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 5 and 7
for i=0:floor(backLenght/gapSize)
	deltaL=backLenght-gapSize*i;
	x=floor(xIn-deltaL*cosH+RobotBackWidth*sinH);
	y=floor(yIn-deltaL*sinH-RobotBackWidth*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available back right side(5-7) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available back right side(5-7) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available back right side(5-7) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 6 and 8
for i=0:floor(backLenght/gapSize)
	deltaL=backLenght-gapSize*i;
	x=floor(xIn-deltaL*cosH-RobotBackWidth*sinH);
	y=floor(yIn-deltaL*sinH+RobotBackWidth*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available back left side(6-8) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available back left side(6-8) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available back left side (6-8) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor


% check availability between 7 and 9
for i=0:floor(RobotBackWidth/gapSize)
	deltaL=RobotBackWidth-gapSize*i;
	x=floor(xIn-backLenght*cosH+deltaL*sinH);
	y=floor(yIn-backLenght*sinH-deltaL*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available back right(7-9) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif

	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available back right(7-9) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available back right(7-9) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor
% check availability between 8 and 9
for i=0:floor(RobotBackWidth/gapSize)
	deltaL=RobotBackWidth-gapSize*i;
	x=floor(xIn-backLenght*cosH-deltaL*sinH);
	y=floor(yIn-backLenght*sinH+deltaL*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available back left(8-9) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif

	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available back left(8-9) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
	if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available back left(8-9) X:%d Y:%d orientation deg: %f .\n",x,y,heading*180/pi())
		endif
		return
	endif
endfor

endfunction