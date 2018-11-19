function [apRobot,robot] = GmapBuild(apRobot,robot,scan)
  GridMap=apGet(apRobot,"GmapGrid");
  PoccMax=apGet(apRobot,"GmapPoccMax");
  PoccMin=apGet(apRobot,"GmapPoccMin");
  PemptyMax=apGet(apRobot,"GmapPemptyMax");
  PemptyMin=apGet(apRobot,"GmapPemptyMin");
  BeamWidth=apGet(apRobot,"GmapBeamWidth");
  XNOref=apGet(apRobot,"currentNorthOrientationReference"); % compass reference for X orientation
  servoRobotHeading=apGet(apRobot,"servoRobotHeading");
  nbRec=size(scan,2);
  # scan (X,Y,NO,scanAngle,scanDistF,scanDistB,cartoId)
  
  for (i=1:nbRec)
    
    
  endfor  
endfunction