function [apRobot,robot,retCode] = UpdateLocationFromNarrowPath(apRobot,robot,zoneNumber,pathDistances,pathEchos,farther,forward)
    retCode=0;
    shift=15;
   zones = GmapNarrowPath();
   path=zones(zoneNumber,:);
   pathOrientation=path(6);
   if (farther)
     pathX1=path(1);
     pathX2=path(2);
     pathY1=path(3);
     pathY2=path(4);
   else
      if(pathOrientation==0)
       pathX1=path(2);
       pathX2=path(1);
       pathY1=path(3);
       pathY2=path(4);
     endif
      if(pathOrientation==90)
       pathX1=path(1);
       pathX2=path(2);
       pathY1=path(4);
       pathY2=path(3);
     endif
   endif
   if(!forward)
      pathEchos=fliplr(pathEchos);
   endif
   pathEchos=[pathEchos,[sum(pathEchos,2)]]
   pathOrientation=path(6);
   pathWidth=path(7);
   pathLength=path(8);
   pathEchos=[pathEchos,[pathEchos(:,3)>pathWidth*.95],[pathEchos(:,3)<pathWidth*1.05]];
   pathEchos=[pathEchos,[pathEchos(:,4).*pathEchos(:,5)]];
   idx = find (pathEchos(:,6));
   newPathEchos=[];
   for (i=1:size(idx,1))
     newPathEchos=[newPathEchos;[pathEchos(idx(i),1:3)]];
  endfor
  if (size(newPathEchos)==0)
     printf(mfilename);
    printf(" echos not compatible with pathWidth:%d *** ",pathWidth);
    printf(ctime(time()));
    retCode=-1;
    return
  endif
  echo1=newPathEchos(1,1);
  trendEcho1=0;
  echo2=newPathEchos(1,2);
  trendEcho2=0;
   for (i=2:size(newPathEchos,1))
       if (newPathEchos(i,1)>echo1)
          trendEcho1++;
       endif
        if (newPathEchos(i,1)<echo1)
          trendEcho1--;
       endif
        if (newPathEchos(i,2)>echo2)
          trendEcho2++;
       endif
        if (newPathEchos(i,2)<echo2)
          trendEcho2--;
       endif
       echo1=newPathEchos(i,1);
       echo2=newPathEchos(i,2);
   endfor
   trendEcho1
   trendEcho2
  newPathEchos
  delta1=newPathEchos(1,1)-newPathEchos(size(newPathEchos,1),1)
  delta2=newPathEchos(1,2)-newPathEchos(size(newPathEchos,1),2)
   if (abs(trendEcho1)>3 && abs(trendEcho2)>3 && (abs(delta1+delta2)>5 || abs(trendEcho1+trendEcho2))>3)
          relatifHeading=0;
          x=pathX2+shift;
          y=round(pathY1+mean(newPathEchos(:,1)))
     else
          relatifHeading=round(atan2(mean([abs(delta1),abs(delta2)]),pathLength)*180/pi())
          x=pathX2+shift
          y=round(pathY1+(newPathEchos(size(newPathEchos,1),1)))       
  endif
  if (delta2<0 && delta1>0)
    relatifHeading=-relatifHeading;
  endif
  if(delta2<0 && delta1<0 || delta2>0 && delta1>0 )
    relatifHeading=0
  endif
    if (farther)
        heading=pathOrientation+relatifHeading;
    else
          heading=180-relatifHeading;
    endif
    apRobot=setfield(apRobot,"location",[x,y,heading]);
 endfunction