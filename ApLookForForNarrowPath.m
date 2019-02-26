function [pathFound,number,entryPoint,pathHeading,pathDistance,farDistances] = ApLookForForNarrowPath(apRobot,robot)
  %{
 
  %}
     servofrontRef=7; % front scan number 90°
     location=apGet(apRobot,"location");
     carto=apGet(apRobot,"carto");
     shitfCartoX=apGet(apRobot,"shitfCartoX");
     shitfCartoY=apGet(apRobot,"shitfCartoY");
     nbPulse=apGet(apRobot,"nbPulse");
     shiftServo=180/(nbPulse-1); % shift between 2 scan in °
     servofrontRef=mod(round((mod(location(3),360)+90)/shiftServo),nbPulse-1);  % find the closest scan angle corresponding to the current orientation
     searchRadius=60;
     narrowPathValue=20;
     points=[];
     pathFound=false;
     number=0;
     pathHeading=0;
     pathDistance=-1;
     farDistances=-1;
     pathOrientation=0;
     entryPoint=[-1,-1];
     for i=-searchRadius:searchRadius
       if(location(1)+shitfCartoX+i>=1 && location(1)+shitfCartoX+i <=size(carto,1))
             for j=-searchRadius:searchRadius 
                if(location(2)+shitfCartoY+j>=1 && location(2)+shitfCartoY+j<=size(carto,2))
                  if(carto(location(1)+shitfCartoX+i,location(2)+shitfCartoY+j)==narrowPathValue)          
                    points=[points;[location(1)+i,location(2)+j,sqrt(i^2+j^2)]];
                    pathFound=true;
                  endif
                endif
              endfor
       endif
     endfor
     if (pathFound)
      [idx1,idx2]=min(points);
      entryPoint=points(idx2(3),1:2);
      pathDistance=points(idx2(3),3);
      pathHeading=mod(atan2(entryPoint(2)-location(2),entryPoint(1)-location(1))*180/pi(),360);
      [number,type,pathOrientation,width,length,northHeadingF,northHeadingC] = GmapZones(apRobot,robot,entryPoint);
     else
      return
     endif
     [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,entryPoint);
      echos=[];
     for (i=1:20)
        robot.GetScanRawData(scanRef(1),scanRef(2));
        if(mod(location(3),360)<=180)
          echos=[echos;[robot.GetScanDistFront(servofrontRef),robot.GetScanDistBack(servofrontRef)]];
        else
          echos=[echos;[robot.GetScanDistBack(round(mod(servofrontRef,nbPulse-1))),robot.GetScanDistFront(round(mod(servofrontRef,nbPulse-1)))]];
        endif
     endfor
      echos(echos==0)=getSensorDistanceLimit();
      farDistances=round(mean(echos));
     return
endfunction