function [cost] = AStarHeuristic(currentX,currentY,targetX,targetY)
%[cost,idx]=max([abs(currentX-targetX),abs(currentY-targetY)]);
cost=sqrt((currentX-targetX)^2+(currentY-targetY)^2);
%cost=cost-.1;
endfunction