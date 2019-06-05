function [instructionNumber,parameters,retCode] = GmapZone4(currentLocation,nextLocation)
    if (currentLocation(1:2)==nextLocation(1:2))
        retcode=-1;
        printf(mfilename);
        printf(" error current and next location are identical *** ");
        printf(ctime(time()));
        return
    endif
    retCode=0;
    instructionNumber=0;
    parameters=[];
    cross=false;
    point1=[540,200];  %540 250 
    point2=[610,300]; %610 350
    optimalCrossX=[560,580];
    line1=createLine(currentLocation(1:2),nextLocation(1:2));
    line2=createLine(point1,point2);
    crossPoint=(intersectLines(line1,line2));
    lineHeading=atan2(point2(2)-point1(2),point2(1)-point1(1))  ;
    if (crossPoint(1)==Inf)
        return
    endif
    dist=round(sqrt((crossPoint(1)-currentLocation(1))^2+(crossPoint(2)-currentLocation(2))^2));
    lenMove=round(sqrt((nextLocation(1)-currentLocation(1))^2+(nextLocation(2)-currentLocation(2))^2));
    sign1=sign(nextLocation(1)-currentLocation(1));
    sign2=sign(nextLocation(1)-crossPoint(1));
    
    if (crossPoint(1)>=point1(1) && crossPoint(1)<=point2(1) && dist <lenMove && sign1==sign2)
        cross=true;
        instructionNumber=1;
        parameters=[round(crossPoint),dist,lineHeading,optimalCrossX];
        printf(mfilename);
        printf(" traject cross line zone :(%d,%d) distance:%d headingLine:%d*** ",crossPoint(1),crossPoint(2),dist,lineHeading*180/pi);
        printf(ctime(time()));
    endif   
    return
 endfunction