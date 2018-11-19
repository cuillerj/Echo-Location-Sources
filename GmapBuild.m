%{
[apRobot,robot] = GmapInit(apRobot,robot)
extract scan with scanRobot.GmapBuild.sql
scan= csvread('filename.csv');
%}
function [apRobot,robot,GridMap] = GmapBuild(apRobot,robot,scan)
  [apRobot,robot] = GmapInit(apRobot,robot);
  GridMap=apGet(apRobot,"GmapGrid");
  [sizeX,sizeY]=size(GridMap);
  PoccMax=apGet(apRobot,"GmapPoccMax");
  PoccMin=apGet(apRobot,"GmapPoccMin");
  PemptyMax=apGet(apRobot,"GmapPemptyMax");
  PemptyMin=apGet(apRobot,"GmapPemptyMin");
  BeamWidth=apGet(apRobot,"GmapBeamWidth")*pi()/180;
  XNOref=apGet(apRobot,"currentNorthOrientationReference"); % compass reference for X orientation
  servoRobotHeading=apGet(apRobot,"servoRobotHeading");
  nbRec=size(scan,1);
  printf("number of records:%d \n",nbRec);
  count=0;
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  # scan (X,Y,NO,scanAngle,scanDistF,scanDistB,cartoId)
  % (x-posX)^2+(y-posY)2=scanDistF^2
  for (i=1:nbRec)
    count++;
    posX=scan(i,1);
    posY=scan(i,2);
    angle=scan(i,3);
    distF=scan(i,4);
    distB=scan(i,5);
    %NO=scan(i,6);
    NO=XNOref;
    beamHeading=(((XNOref-NO)+(angle-servoRobotHeading)))*pi()/180;
    if (mod(count,10)==0)    
      printf("-%d",count);
    endif
    if (mod(count,100)==0)    
      printf("\n",count);
    endif
    % empty front
    [GridMap] = GmapEmptyGrid(apRobot,GridMap,posX+shitfCartoX,posY+shitfCartoY,distF,beamHeading,angle,BeamWidth);
    [GridMap] = GmapEmptyGrid(apRobot,GridMap,posX+shitfCartoX,posY+shitfCartoY,distB,mod(beamHeading+pi(),2*pi()),angle,BeamWidth);
    % empty back
    [GridMap] = GmapOccupiedGrid(apRobot,GridMap,posX+shitfCartoX,posY+shitfCartoY,distF,mod(beamHeading,2*pi()),BeamWidth);
    [GridMap] = GmapOccupiedGrid(apRobot,GridMap,posX+shitfCartoX,posY+shitfCartoY,distB,mod(beamHeading+pi(),2*pi()),BeamWidth);
  endfor
  printf("\n")
  picture=gray2ind(GridMap',256);
  maxAxis=max(sizeX,sizeY);
  figure()
  hold on;
  axis ([-shitfCartoX, maxAxis-shitfCartoX,-shitfCartoY, maxAxis-shitfCartoY], "square");
  picture=gray2ind(((GridMap.*-1).+1)',64);
  grid minor on ;
  image(picture);
  hold off;
 % saveas (1, "carto-1.pdf");
  saveas (2, "carto-2.pdf");
endfunction