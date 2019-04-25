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
  zones = GmapNarrowPath();
  number=0;
  type=0;
  heading=0;
  width=0;
  length=0;
  northHeadingF=0;
  northHeadingC=0;
  for i=1:size(zones,1)
    if (X>=zones(i,1)  && X<=zones(i,2) && Y>=zones(i,3)  && Y<=zones(i,4))
      number=i
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
  