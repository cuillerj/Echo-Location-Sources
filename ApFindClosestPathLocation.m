function [apRobot,robot,closestPathPoint,closestIdx] = ApFindClosestPathLocation(apRobot,robot,location)
%{ 
 find the closest points located on the optimal path
%}
  pathStep=apGet(apRobot,"pathStep")(:,1:2);
  currentX=location(1);
  currentY=location(2);
	prevDist=inf;
	closestIdx=0;
	for i=1:size(pathStep,1)
		dist=(pathStep(i,1)-currentX)^2+(pathStep(i,2)-currentY)^2;
		if (dist<=prevDist)
				prevDist=dist;
        closestIdx=i;
		endif
	endfor
  closestPathPoint=apGet(apRobot,"optimalPath")(closestIdx,:)
	printf(mfilename);
	printf(" *** closest step is: X:%d Y:%d H:%d. *** ", closestPathPoint(1),closestPathPoint(2),closestPathPoint(3));
	printf(ctime(time()));

 % currentL=apGet(apRobot,"location");
 % currentX=currentL(1);
%  currentY=currentL(2);
endfunction