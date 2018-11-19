function [GridMap] = GmapEmptyGrid(apRobot,GridMap,posX,posY,dist,beamHeading,angle,BeamWidth)
    PemptyMax=apGet(apRobot,"GmapPemptyMax");
    PemptyMin=apGet(apRobot,"GmapPemptyMin");
    PoccMax=apGet(apRobot,"GmapPoccMax");
    PoccMin=apGet(apRobot,"GmapPoccMin");
    delatEmpty=PoccMax-PemptyMin;
%    xC=round(posX+dist*cos(beamHeading));
%    yC=round(posY+dist*sin(beamHeading));
    range=[beamHeading-BeamWidth/2:pi()/720:beamHeading+BeamWidth/2];
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
    minH=mod((beamHeading-BeamWidth/2),2*pi());
    maxH=mod((beamHeading+BeamWidth/2),2*pi());
    tMin=tan(beamHeading-BeamWidth/2);
    tMax=tan(beamHeading+BeamWidth/2);
    [sizeX,sizeY]=size(GridMap);
    limX1=ceil(max(min(minx,posX),1));
    limX2=floor(min(max(maxx,posX),sizeX));
    limY1=ceil(max(min(miny,posY),1));
    limY2=floor(min(max(maxy,posY),sizeY));
    SqDist=dist^2;
      for x=limX1:limX2
        for y=limY1:limY2
          if (x-posX!=0)
            tPos=(y-posY)/(x-posX);
             distXY=(x-posX)^2+(y-posY)^2;
             Pvalue=PemptyMin+max(min(delatEmpty*sqrt(distXY)/dist,1),0);
            if (tMin<=tMax)
              if ((tPos>=tMin) && (tPos <=tMax))
                if (distXY<SqDist)
                  GridMap(x,y)=GridMap(x,y)*Pvalue;
                endif
              endif
            else
              if ((tPos<tMax) && tPos<0)
                if (distXY<SqDist)
         %         Pvalue=PemptyMin;
                  GridMap(x,y)=GridMap(x,y)*Pvalue;
                endif
              endif
              if ((tPos>tMin) && tPos>0)
                if (distXY<SqDist)
            %      Pvalue=PemptyMin;
                  GridMap(x,y)=GridMap(x,y)*Pvalue;
                endif
              endif                  
            endif
           else
           if ((angle==00 || angle==180) && (y-posY)<dist)
              Pvalue=PemptyMax-max(min(delatEmpty*abs(posX-x)/dist,1),0);
              GridMap(x,y)=GridMap(x,y)*Pvalue;
             endif
           endif
          endfor
      endfor
      