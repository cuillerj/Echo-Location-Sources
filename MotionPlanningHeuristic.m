function [cost] = MotionPlanningHeuristic(currentX,currentY,currentHeading,targetX,targetY)
% this function must provide cost always <= than MotionCost
miniRotationCost=500; % must be <=  than in motion cost
[rotation,distance]=ComputeMoveToDo(currentX,currentY,currentHeading,targetX,targetY);
motionCost=MotionCost(rotation,distance)
if (rotation!=0)
	cost=distance+miniRotationCost;
else
	cost=distance;
endif
endfunction