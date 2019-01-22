function [apRobot,robot] = ApPlotEchoScanRobot(apRobot,robot)
	%{

%}
	nbPingByScan=apGet(apRobot,"nbPulse");
	shiftAngle=pi()/(nbPingByScan-1);
  scanFront=[];
  scanBack=[];
  for (i=1:nbPingByScan)
    angleR=(i-1)*shiftAngle;
    scanFront=[scanFront,[robot.GetScanDistFront(i-1)]];
    scanBack=[scanBack,[robot.GetScanDistBack(i-1)]];
  endfor
   figureNumber = ApPlotEchoScan(scanFront,scanBack)
    hold off;
endfunction