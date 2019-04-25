function [actionNumber,parameters,retCode] = GmapZone1(currentLocation,nextLocation)
    retCode=0;
    actionNumber=0;
    if (currentLocation(1:2)==nextLocation(1:2))
        retcode=-1
        printf(mfilename);
        printf(" error current and next location are identical *** ");
        printf(ctime(time()));
        return
    endif
    retCode=0;
    actionNumber=0;
    parameters=[];
    narrowPath= GmapNarrowPath()(1,:);
    if (currentLocation(1)>=narrowPath(1)  && currentLocation(1)<=narrowPath(2) && currentLocation(2)>=narrowPath(3)  && currentLocation(2)<=narrowPath(4))
        actionNumber=10; % current location indide the path
        parameters=[narrowPath(5),narrowPath(6),narrowPath(6),narrowPath(8),narrowPath(9),narrowPath(10)]; % [type,heading,width,length,northHeadinFront,northHeadingBack]
    endif
     if (nextLocation(1)>=narrowPath(1)  && nextLocation(1)<=narrowPath(2) && nextLocation(2)>=narrowPath(3)  && nextLocation(2)<=narrowPath(4))
         if (actionNumber==10)
           actionNumber=11; % both current and next location inside narrow path
           else
            actionNumber=12; % next location indide the path
         endif
        parameters=[narrowPath(5),narrowPath(6),narrowPath(6),narrowPath(8),narrowPath(9),narrowPath(10)]; % [type,heading,width,length,northHeadinFront,northHeadingBack]
    endif
    if (actionNumber==0)
       % segment=
       if(nextLocation(1)!=currentLocation(1))
            a=(nextLocation(2)-currentLocation(2))/(nextLocation(1)-currentLocation(1));
            b=-a*currentLocation(1)+currentLocation(2);
        else
            a=0;
            b=currentLocation(2);
        endif
       function y=f(x,a,b) % define traject line equation
           y=a*x+b;
        endfunction
        cross=false;
        minDist=Inf;
        for (x=narrowPath(1):narrowPath(2))
            y=f(x,a,b);
            if (y>=narrowPath(3) && y<=narrowPath(4))  % look for common point
                cross=true;
               actionNumber=13;
                dist=(x-currentLocation(1))^2+(y-currentLocation(2));
                if (dist<minDist)
                    minDist=dist;
                    crossPoint=[x,y];
                endif
            endif
        endfor
        if ( minDist >(nextLocation(1)-currentLocation(1))^2+(nextLocation(2)-currentLocation(2))^2)
            cross=false;
        endif
         if(cross)                  
            minDist=sqrt(minDist);
            crossPoint=round(crossPoint);
            printf(mfilename);
            printf(" traject cross narrow path:(%d,%d) distance:%d *** ",crossPoint(1),crossPoint(2),round(minDist))
            printf(ctime(time()));
        endif
    endif
    actionNumber=0; % temporaly forced
    return
 endfunction