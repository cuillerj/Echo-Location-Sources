function [number,type,heading,width,length,northHeadingF,northHeadingC] = GmapZones(apRobot,robot,location)
  %{
  [xmin,xmax,ymin,ymax,type,heading,width,length,northHeadingF,northHeadingC]
  xmin,xmax,ymin,ymax (x,y) limits
  type: 1 door
  width of the pass
  lengh of the pass
  nortHeadingF when going thru the pass farther from (x=0,y=0) (x^2+y^2) increasing
 nortHeadingC when going thru the pass closer from (x=0,y=0) (x^2+y^2) decreasing
  %}
  X=location(1);
  Y=location(2);
  zones=[420,440,210,280,1,0,78,35,304,124;
              1,20,31,130,1,0,78,35,160,340;
              210,270,340,360,1,90,78,35,308,128];
  number=0;
  type=0;
  heading=0;
  for i=1:size(zones,1)
    if (X>=zones(i,1)  && X<=zones(i,2) && Y>=zones(i,3)  && Y<=zones(i,4))
      number=i;
      break
    endif
  endfor
  if (number>0)
    type=zones(number,5);
    heading=zones(number,6);
    width=zones(number,7);
    length=zones(number,8);
    northHeadingF=zones(number,9);
    northHeadingC=zones(number,10);
  endif
  endfunction
  