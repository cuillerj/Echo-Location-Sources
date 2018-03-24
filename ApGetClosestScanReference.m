 function [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,location)
   %{
   this function return the closest reference scan point to a location and the distance 
   between the reference and the location
   %}
   scanRefPoints=apGet(apRobot,"scanRefPoints");
   [nbPoints,width]=size(scanRefPoints);
   distance=[];
   for (i=1:nbPoints)
     distance=[distance;[(scanRefPoints(i,1)-location(1))^2+(scanRefPoints(i,2)-location(2))^2]];
     i++;
   end
   [distance,idx]=min(distance);
   distance=sqrt(distance);
   scanRef=scanRefPoints(idx,1:3);
 endfunction