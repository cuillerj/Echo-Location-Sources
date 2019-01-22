function [figureNumber] = ApPlotEchoScan(scanFront,scanBack)
	col=['r','g','b','m','c']; % graph color 
	figureNumber=figure();
  maxDistance=getSensorDistanceLimit();
	title ("Scan 360 (red:ping front <=> blue:ping back)");
	ylabel ("Y: is robot front orientation");
	axis ("manual",[-maxDistance,maxDistance,-maxDistance,maxDistance])
	hold on;
  scanFront=(scanFront==0)*maxDistance+scanFront;
  scanBack=(scanBack==0)*maxDistance+scanBack;
	%[param,value,found]=ApeRobotCommonDefine("nbPulse");
	nbPingByScan=getNbStepsRotation();
	shiftAngle=pi()/(nbPingByScan-1);
	plot (0,0,"color","k","o","markersize",15);
	legend ("robot location","location","northeast");
  for (i=1:nbPingByScan)
    angleR=(i-1)*shiftAngle;
    X=scanFront(i)*cos(angleR);
    Y=scanFront(i)*sin(angleR);
    plot (X,Y,col(1),"markersize",15);
    X=-scanBack(i)*cos(angleR);
    Y=-scanBack(i)*sin(angleR);
    plot (X,Y,col(3),"markersize",15);
  endfor


	hold off;
endfunction