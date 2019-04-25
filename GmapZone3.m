function [actionNumber,parameters,retCode] = GmapZone3(currentLocation,nextLocation)
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
 endfunction