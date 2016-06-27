function [available] = QueryCartoAvailability(xIn,yIn,heading,cartoId,debugOn)
% multiple cartoId to be developped
% take into account the robot physical characteristics (by checking 10 points) to determine if it can be at a place
cartoMaxAvailableValue=20;
[Param,Value,Found] = ApeRobotCommonDefine("iRobotWidth"); 
cosH=cos(heading);
sinH=sin(heading);
if (Found==true)
	RobotBackWidth = Value/2;
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
x=floor(xIn+frontLenght/2*cosH+RobotBackWidth*sinH);
y=floor(yIn+frontLenght/2*sinH-RobotBackWidth*cosH);
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
x=floor(xIn-backLenght/2*cosH-RobotBackWidth*sinH);
y=floor(yIn-backLenght/2*sinH+RobotBackWidth*cosH);
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
endfunction