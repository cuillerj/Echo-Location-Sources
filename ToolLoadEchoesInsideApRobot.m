 function [apRobot,robot] = ToolLoadEchoesInsideApRobot(apRobot,robot,scan)
   %{
   maintenance and development tool
   load apRobot with data scan (heading,X,Y) sorted by heading (asc or desc)
   %}
  [l,c]=size(scan)
  if (l!=15 || c!=3)
    printf("incorrect scan\n")
    return
  endif
  if (scan(1,1)==180)
    scan=flipud(scan);
  endif
  for i=1:l
    robot.SetScanDistFront(i-1,scan(i,2));
    robot.SetScanDistBack(i-1,scan(i,3));
  endfor
endfunction