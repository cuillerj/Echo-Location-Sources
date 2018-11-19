function [apRobot,robot] = GmapInit(apRobot,robot)
  PoccMax=0.9;
  PoccMin=0.6;
  PemptyMax=0.4;
  PemptyMin=0.1;
  BeamWidth=12;
  apGet(apRobot,"shitfCartoX");
  apGet(apRobot,"shitfCartoY");
  GridMap=ones(900+  apGet(apRobot,"shitfCartoX"),600+  apGet(apRobot,"shitfCartoY"))*.5;  % (x,y)
  apRobot = setfield(apRobot,"GmapGrid",GridMap);   
  apRobot = setfield(apRobot,"GmapPoccMax",PoccMax);
  apRobot = setfield(apRobot,"GmapPoccMin",PoccMin);   
  apRobot = setfield(apRobot,"GmapPemptyMax",PemptyMax);
  apRobot = setfield(apRobot,"GmapPemptyMin",PemptyMin); 
  apRobot = setfield(apRobot,"GmapBeamWidth",BeamWidth); 
endfunction