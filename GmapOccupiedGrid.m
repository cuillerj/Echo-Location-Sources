function [GridMap] = GmapOccupiedGrid(apRobot,GridMap,posX,posY,dist,beamHeading,BeamWidth)
    if (dist<=0)
      return
    endif
    PoccMax=apGet(apRobot,"GmapPoccMax");
    PoccMin=apGet(apRobot,"GmapPoccMin");
    minH=mod((beamHeading-BeamWidth/2),2*pi());
    maxH=mod((beamHeading+BeamWidth/2),2*pi());
    range=[beamHeading-BeamWidth/2:pi()/720:beamHeading+BeamWidth/2];
    deltaOcc=1.6;
    Xrange=cos(range)*dist;
    [maxx,b]=max(Xrange);
    [minx,b]=min(Xrange);
    maxx=maxx+posX;
    minx=minx+posX;
    Yrange=sin(range)*dist;
    [maxy,b]=max(Yrange);
    [miny,b]=min(Yrange);
    maxy=maxy+posY;
    miny=miny+posY;
    center=[posX+dist*cos(beamHeading),posY+dist*sin(beamHeading)];
    pointMin=[posX+dist*cos(minH),posY+dist*sin(minH)];
 %   dCenter=sqrt((center(1)-pointMin(1))^2+(center(2)-pointMin(2))^2);
  %  pointMax=[posX+dist*cos(maxH),posY+dist*sin(maxH)]
    arcLen=sqrt((center(1)-pointMin(1))^2+(center(2)-pointMin(2))^2);
    [sizeX,sizeY]=size(GridMap);
    limX1=ceil(max(min(minx,posX),1));
    limX2=floor(min(max(maxx,posX),sizeX));
    limY1=ceil(max(min(miny,posY),1));
    limY2=floor(min(max(maxy,posY),sizeY));
  % (x-posX)^2+(y-posY)2=scanDistF^2
    if (beamHeading>=pi()/4 && beamHeading< 3*pi()/4)
 %     printf("case 1\n");
      for x=limX1:limX2
          y=round(sqrt((dist^2)-(x-posX)^2))+posY;
          if (y>=limY1 && y<=limY2)
            for (x=max(1,x-1):min(sizeX,x+1))
              for (y=max(1,y-1):min(sizeY,y+1))
                Pvalue=0.5+max(PoccMax-deltaOcc*sqrt((x-center(1))^2+(y-center(2))^2)/arcLen,PoccMin);
                GridMap(x,y)=min((GridMap(x,y))*Pvalue,1);
               endfor
            endfor
          endif
      endfor
    endif
   if (beamHeading>=5*pi()/4 && beamHeading< 7*pi()/4)
 %     printf("case 2\n");
      for x=limX1:limX2
          y=-round(sqrt((dist^2)-(x-posX)^2))+posY;
          if (y>=limY1 && y<=limY2)
            for (x=max(1,x-1):min(sizeX,x+1))
              for (y=max(1,y-1):min(sizeY,y+1))
                Pvalue=0.5+max(PoccMax-deltaOcc*sqrt((x-center(1))^2+(y-center(2))^2)/arcLen,PoccMin);
                GridMap(x,y)=min((GridMap(x,y))*Pvalue,1);
               endfor
            endfor
          endif
      endfor
    endif
    if (beamHeading>=0&& beamHeading< pi()/4)
  %    printf("case 3\n");
      for y=limY1:limY2
          x=round(sqrt((dist^2)-(y-posY)^2))+posX;
          if (x>=limX1 && x<=limX2)
            for (x=max(1,x-1):min(sizeX,x+1))
              for (y=max(1,y-1):min(sizeY,y+1))
                Pvalue=0.5+max(PoccMax-deltaOcc*sqrt((x-center(1))^2+(y-center(2))^2)/arcLen,PoccMin);
                GridMap(x,y)=min((GridMap(x,y))*Pvalue,1);
               endfor
            endfor
          endif
      endfor
    endif
    if (beamHeading< 7*pi()/4)
 %     printf("case 4\n");
      for y=limY1:limY2
          x=-round(sqrt((dist^2)-(y-posY)^2))+posX;
          if (x>=limX1 && x<=limX2)
            for (x=max(1,x-1):min(sizeX,x+1))
              for (y=max(1,y-1):min(sizeY,y+1))
                Pvalue=0.5+max(PoccMax-deltaOcc*sqrt((x-center(1))^2+(y-center(2))^2)/arcLen,PoccMin);
                GridMap(x,y)=min((GridMap(x,y))*Pvalue,1);
               endfor
            endfor
          endif
      endfor
    endif
endfunction