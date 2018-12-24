function [rotation] = ToolOptimizeNorthAlign(currentNO,targetNO)
  %{
  compute the optimum rotation for compas alignement from current and target north orientation)
  positive north roation is clockwise and poistion rotation counte-rclockwise
  %}
  
  delta = ToolAngleDiff(currentNO,targetNO)
  if (delta > 180)
    rotation=ToolAngleDiff(targetNO,currentNO);
  else
    rotation=-delta;
  endif
endfunction