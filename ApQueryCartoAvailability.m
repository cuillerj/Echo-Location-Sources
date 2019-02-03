function [available,retCode] = ApQueryCartoAvailability(apRobot,location,radian,debugOn,fast)
% fast mode check only one point
% if radian false then heading expressed in degres
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
 if (!exist("fast"))  % fast mode
      fast=false;
 endif
shitfCartoX=apGet(apRobot,"shitfCartoX");
shitfCartoY=apGet(apRobot,"shitfCartoY");
xIn=location(1)+shitfCartoX;
yIn=location(2)+shitfCartoY;
if (radian==false)
    heading=location(3)*pi()/180;
else
    heading=location(3);
endif
carto=apGet(apRobot,"carto");
cartoMaxAvailableValue=apGet(apRobot,"cartoMaxAvailableValue");       % over this value location is not possible
iRobotWidth=apGet(apRobot,"iRobotWidth");
RobotBackWidth = iRobotWidth/2;
stepSize=apGet(apRobot,"stepSize");
gapSize=stepSize-1;
iRobotFrontWidth=apGet(apRobot,"iRobotFrontWidth");
RobotFrontWidth=iRobotFrontWidth/2;
frontLenght=apGet(apRobot,"frontLenght");
backLenght=apGet(apRobot,"backLenght");
cosH=cos(heading);
sinH=sin(heading);
xIn=round(xIn);
yIn=round(yIn);
[cartoX,cartoY]=size(carto);
available=true;
x=xIn;
y=yIn;
retCode=0;

if (x <= 0 || x > cartoX)
	available=false;
  if(debugOn)
    printf(mfilename);
    printf(" *** carto not available:%d (1.1) X:%d Y:%d orientation deg: %f . ",carto(x,y),x,y,heading*180/pi())
    printf(ctime(time()))
  endif
	retCode=1;
	return
endif
if (y <= 0 || y > cartoY)
	available=false;
  if(debugOn)
    printf(mfilename);
    printf(" *** carto not available:%d (1.2) X:%d Y:%d orientation deg: %f . ",carto(x,y),x,y,heading*180/pi())
    printf(ctime(time()))
  endif
	retCode=2;
	return
endif
try
    if (carto(x,y)>cartoMaxAvailableValue)     
      available=false;
      if(debugOn)
      printf(mfilename);
      printf(" *** carto not available:%d (1.3) X:%d Y:%d orientation deg: %f . ",carto(x,y),x,y,heading*180/pi())
      printf(ctime(time()))
      endif
      retCode=3;
      return
    endif
    if (fast)
      return
    endif
    % check availability between 0 and x
    for i=0:round(frontLenght/gapSize)
      deltaL=frontLenght-gapSize*i;
      x=round(xIn+deltaL*cosH);
      y=round(yIn+deltaL*sinH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d center(0-x) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=4;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d center (0-x) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=5;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d center(0-x) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=6;
        return
      endif
    endfor
    % check availability between 9 and x
    for i=0:round(backLenght/gapSize)
      deltaL=backLenght-gapSize*i;
      x=round(xIn-deltaL*cosH);
      y=round(yIn-deltaL*sinH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back(9-x) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=7;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back (9-x) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=8;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back(9-x) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=9;
        return
      endif
    endfor
    % check availability between 0 and 1
    for i=0:round(RobotFrontWidth/gapSize)
      deltaL=RobotFrontWidth-gapSize*i;
      x=round(xIn+frontLenght*cosH+deltaL*sinH);
      y=round(yIn+frontLenght*sinH-deltaL*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d front(0-1) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=10;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d front (0-1) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=11;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d front(0-1) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=12;
        return
      endif
    endfor
    % check availability between 0 and 3
    for i=0:round(RobotFrontWidth/gapSize)
      deltaL=RobotFrontWidth-gapSize*i;
      x=round(xIn+frontLenght*cosH-deltaL*sinH);
      y=round(yIn+frontLenght*sinH+deltaL*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d front(0-3) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=13;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d front(0-3) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=14;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d front(0-3) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=15;
        return
      endif
    endfor
    % check availability between 1 and 2
    for i=0:round(frontLenght/gapSize)
      deltaL=frontLenght-gapSize*i;
      x=round(xIn+deltaL*cosH+RobotFrontWidth*sinH);
      y=round(yIn+deltaL*sinH-RobotFrontWidth*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d right side (1-2) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=16;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d right side (1-2) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=17;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d right side(1-2) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=18;
        return
      endif
    endfor
    % check availability between 3 and 4
    for i=0:round(frontLenght/gapSize)
      deltaL=frontLenght-gapSize*i;
      x=round(xIn+deltaL*cosH-RobotFrontWidth*sinH);
      y=round(yIn+deltaL*sinH+RobotFrontWidth*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d left side(3-4) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=19;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
         printf(mfilename);
          printf(" *** carto not available:%d left side (3-4) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=20;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d left side(3-4) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=21;
        return
      endif
    endfor
    % check availability between 5 and 7
    for i=0:round(backLenght/gapSize)
      deltaL=backLenght-gapSize*i;
      x=round(xIn-deltaL*cosH+RobotBackWidth*sinH);
      y=round(yIn-deltaL*sinH-RobotBackWidth*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back right side(5-7) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=22;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back right side(5-7) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=23;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back right side(5-7) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=24;
        return
      endif
    endfor
    % check availability between 6 and 8
    for i=0:round(backLenght/gapSize)
      deltaL=backLenght-gapSize*i;
      x=round(xIn-deltaL*cosH-RobotBackWidth*sinH);
      y=round(yIn-deltaL*sinH+RobotBackWidth*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back left side(6-8) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=25;
        return
      endif
      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back left side(6-8) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=26;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back left side (6-8) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=27;
        return
      endif
    endfor


    % check availability between 7 and 9
    for i=0:round(RobotBackWidth/gapSize)
      deltaL=RobotBackWidth-gapSize*i;
      x=round(xIn-backLenght*cosH+deltaL*sinH);
      y=round(yIn-backLenght*sinH-deltaL*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back right(7-9) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=28;
        return
      endif

      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back right(7-9) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=29;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back right(7-9) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=30;
        return
      endif
    endfor
    % check availability between 8 and 9
    for i=0:round(RobotBackWidth/gapSize)
      deltaL=RobotBackWidth-gapSize*i;
      x=round(xIn-backLenght*cosH-deltaL*sinH);
      y=round(yIn-backLenght*sinH+deltaL*cosH);
      if (x <= 0 || x > cartoX)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d back left(8-9) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=31;
        return
      endif

      if (y <= 0 || y > cartoY)
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d  back left(8-9) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=32;
        return
      endif
      if (carto(x,y)>cartoMaxAvailableValue)     % check availabity of echo position
        available=false;
        if(debugOn)
          printf(mfilename);
          printf(" *** carto not available:%d  back left(8-9) X:%d Y:%d orientation deg: %f .\n",carto(x,y),x,y,heading*180/pi())
        endif
        retCode=33;
        return
      endif
    endfor
  catch
    possible=false;
    retCode=99;
    printf(mfilename);
    printf(" *** error **** "); 
    printf(ctime(time()))
 end_try_catch 
endfunction