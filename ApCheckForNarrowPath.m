function [pathFound,pathHeading,pathDistance,backDistance] = ApCheckForNarrowPath(apRobot)
     location=apGet(apRobot,"location")
     carto=apGet(apRobot,"carto");
     searchRadius=60;
     carto(location(1:2))
endfunction