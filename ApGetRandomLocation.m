  function [apRobot,robot,randomLocation] = ApGetRandomLocation(apRobot,robot)
   %{
   this function return a random allowed location inside the space
   first select randomly a scan point reference
   apply ramdom gaussian noise to the (x,y) and select a random orientation
   %}
      sigmaDist=2; 
      scanRefPoints=apGet(apRobot,"scanRefPoints");
      [nbPoints,width]=size(scanRefPoints);
      available=false;
      while (!available)
        randomIdx=randi(nbPoints);
        px=round(normrnd(scanRefPoints(randomIdx,1),sigmaDist));
        py=round(normrnd(scanRefPoints(randomIdx,2),sigmaDist));
        ph=round(randi(361)-1);
        randomLocation=[px,py,ph];
        [available,retCode] = ApQueryCartoAvailability(apRobot,randomLocation,0,0);
      end
      robot.simulatedHardX=scanRefPoints(randomIdx,1);  % set simulated hard position to guess 
      robot.simulatedHardY=scanRefPoints(randomIdx,2);  % used by java simulator to generate an echo scan
      robot.simulatedHardH=scanRefPoints(randomIdx,3);
   endfunction