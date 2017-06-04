function [apRobot,robot,closestOptimalPoint,closestIdx] = ApFindClosestOptimalLocation(apRobot,robot,location)
%{ 
 find the closest points located on the optimal path
%}
  AStarStep=apGet(apRobot,"optimalPath")(:,1:2);
  currentX=location(1);
  currentY=location(2);
	prevDist=inf;
	closestIdx=0;
	for i=1:size(AStarStep,1)
		dist=(AStarStep(i,1)-currentX)^2+(AStarStep(i,2)-currentY)^2;
		if (dist<=prevDist)
				prevDist=dist;
        closestIdx=i;
		endif
	endfor
  closestOptimalPoint=apGet(apRobot,"optimalPath")(closestIdx,:)
	printf(mfilename);
	printf(" *** closest optimal step is: X:%d Y:%d H:%d. *** ", closestOptimalPoint(1),closestOptimalPoint(2),closestOptimalPoint(3));
	printf(ctime(time()));

%  currentL=apGet(apRobot,"location");
 % currentX=currentL(1);
 % currentY=currentL(2);

endfunction