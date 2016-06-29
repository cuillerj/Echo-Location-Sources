function [available] = QueryCartoAvailability(xIn,yIn,heading,cartoId,debugOn)
% multiple cartoId to be developped
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
	sinH=sin(heading)
	(x0,y0) = (x + F*sinH , y + F*cosH)
	(x1,y1) = (x + F*sinH + (iRobotFrontWidth/2)*sinH , y + F*cosH - (iRobotFrontWidth/2)*cosH)
	(x3,y3) = (x + F*sinH - (iRobotFrontWidth/2)*sinH , y + F*cosH + (iRobotFrontWidth/2)*cosH)
	(x2,y2) = (x + (iRobotFrontWidth/2)*sinH , y - (iRobotFrontWidth/2)*cosH)
	(x4,y4) = (x - (iRobotFrontWidth/2)*sinH , y + (iRobotFrontWidth/2)*cosH)
	(x5,y5) = (x + (iRobotWidth/2)*sinH , y - (iRobotWidth/2)*cosH)
	(x6,y6) = (x - (iRobotWidth/2)*sinH , y + (iRobotWidth/2)*cosH)
	(x9,y9) = (x - B*cosH , y - B*CosH)
	(x7,y7) = (x - B*cosH + (iRobotWidth/2)*sinH , y - B*CosH - (iRobotWidth/2)*cosH)
	(x8,y8) = (x - B*cosH - (iRobotWidth/2)*sinH , y - B*CosH + (iRobotWidth/2)*cosH)
		
%}
cartoMaxAvailableValue=20;
[Param,Value,Found] = ApeRobotCommonDefine("iRobotWidth"); 
cosH=cos(heading);
sinH=sin(heading);
if (Found==true)
	RobotBackWidth = Value/2;
endif
[Param,Value,Found] = ApeRobotCommonDefine("stepSize"); 
cosH=cos(heading);
sinH=sin(heading);
if (Found==true && Value>1)
	gapSize = Value-1;
endif
[Param,Value,Found] = ApeRobotCommonDefine("iRobotFrontWidth"); 
cosH=cos(heading);
sinH=sin(heading);
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
if (cartoId==1)
	load carto1;
else
	load carto0;
endif 
[cartoX,cartoY]=size(carto1);
available=true;
x=xIn;
y=yIn;
if (x <= 0 || x > cartoX)
	available=false;
	printf("carto not available (1) X:%d Y:%d orientation: %f .\n",x,y,heading)
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	printf("carto not available (1) X:%d Y:%d orientation: %f .\n",x,y,heading)
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
	printf("carto not available (1) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
x=floor(xIn+frontLenght*cosH-RobotFrontWidth*sinH);
y=floor(yIn+frontLenght*sinH+RobotFrontWidth*cosH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
	printf("carto not available (2) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
	printf("carto not available (2) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
	printf("carto not available (2) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
%{
x=floor(xIn+frontLenght*cosH+RobotBackWidth*sinH);
y=floor(yIn+frontLenght*sinH-RobotBackWidth*cosH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
	printf("carto not available front (3) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available front(3) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
	printf("carto not available front (3) X:%d Y:%d orientation: %f. .\n",x,y,heading)
	endif
	return
endif

x=floor(xIn-backLenght*cosH-RobotBackWidth*sinH);
y=floor(yIn-backLenght*sinH+RobotBackWidth*cosH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
		printf("carto not available back(4) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif

if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available back (4) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
		printf("carto not available back(4) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
%}
x=floor(xIn-backLenght*cosH+RobotBackWidth*sinH);
y=floor(yIn-backLenght*sinH-RobotBackWidth*cosH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
		printf("carto not available back(5) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available back (5) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
		printf("carto not available back (5) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
x=floor(xIn+frontLenght*cosH);
y=floor(yIn+frontLenght*sinH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
	printf("carto not available front (6) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available front(6) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
	printf("carto not available front (6) X:%d Y:%d orientation: %f. .\n",x,y,heading)
	endif
	return
endif
x=floor(xIn-backLenght*cosH);
y=floor(yIn-backLenght*sinH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
		printf("carto not available back(7) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif

if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available back (7) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
		printf("carto not available back(7) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
x=floor(xIn+RobotBackWidth*sinH);
y=floor(yIn-RobotBackWidth*cosH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
	printf("carto not available front (8) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available front(8) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
	printf("carto not available front (8) X:%d Y:%d orientation: %f. .\n",x,y,heading)
	endif
	return
endif
x=floor(xIn-RobotBackWidth*sinH);
y=floor(yIn+RobotBackWidth*cosH);
if (x <= 0 || x > cartoX)
	available=false;
	if(debugOn)
		printf("carto not available back(9) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif

if (y <= 0 || y > cartoY)
	available=false;
	if(debugOn)
		printf("carto not available back (9) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
	available=false;
	if(debugOn)
		printf("carto not available back(9) X:%d Y:%d orientation: %f .\n",x,y,heading)
	endif
	return
endif
for i=0:floor(frontLenght/gapSize)
	deltaL=frontLenght-gapSize*i
	x=floor(xIn+deltaL*cosH+RobotBackWidth*sinH);
	y=floor(yIn+deltaL*sinH-RobotBackWidth*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available front (10) X:%d Y:%d orientation: %f .\n",x,y,heading)
		endif
		return
	endif
	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available front(10) X:%d Y:%d orientation: %f .\n",x,y,heading)
		endif
		return
	endif
	if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available front (10) X:%d Y:%d orientation: %f. .\n",x,y,heading)
		endif
		return
	endif
endfor

for i=0:floor(backLenght/gapSize)
	deltaL=backLenght-gapSize*i
	x=floor(xIn-deltaL*cosH-RobotBackWidth*sinH);
	y=floor(yIn-deltaL*sinH+RobotBackWidth*cosH);
	if (x <= 0 || x > cartoX)
		available=false;
		if(debugOn)
			printf("carto not available back(4) X:%d Y:%d orientation: %f .\n",x,y,heading)
		endif
		return
	endif

	if (y <= 0 || y > cartoY)
		available=false;
		if(debugOn)
			printf("carto not available back (4) X:%d Y:%d orientation: %f .\n",x,y,heading)
		endif
		return
	endif
	if (carto1(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
		available=false;
		if(debugOn)
			printf("carto not available back(4) X:%d Y:%d orientation: %f .\n",x,y,heading)
		endif
		return
	endif
endfor
endfunction