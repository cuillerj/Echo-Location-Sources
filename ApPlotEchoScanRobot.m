function [apRobot,robot] = ApPlotEchoScanRobot(apRobot,robot)
	col=['r','g','b','m','c']; % graph color 
	figure();
	title ("Last scan 360 (red:ping front <=> blue:ping back)");
	ylabel ("Y: is robot front orientation");
	axis ("manual",[-400,400,-400,400])
	hold on;
	%[param,value,found]=ApeRobotCommonDefine("nbPulse");
	nbPingByScan=apGet(apRobot,"nbPulse");
	shiftAngle=pi()/(nbPingByScan-1);
	plot (0,0,"color","k","o","markersize",15);
	legend ("robot location","location","northeast");
for (i=1:nbPingByScan)
	angleR=(i-1)*shiftAngle;
	DF=robot.GetScanDistFront(i-1);
	DB=robot.GetScanDistBack(i-1);
	X=DF.*cos(angleR);
	Y=DF.*sin(angleR);
	plot (X,Y,col(1));
	X=-DB.*cos(angleR);
	Y=-DB.*sin(angleR);
	plot (X,Y,col(3));
end

	hold off;
endfunction