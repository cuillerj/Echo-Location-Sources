function [zones] = GmapNarrowPath()
    %{
  [xmin,xmax,ymin,ymax,type,heading,width,length,northHeadingF,northHeadingC]
  xmin,xmax,ymin,ymax (x,y) limits
  type: 1 door
  width of the pass
  lengh of the pass
  nortHeadingF when going thru the pass farther from (x=0,y=0) (x^2+y^2) increasing
 nortHeadingC when going thru the pass closer from (x=0,y=0) (x^2+y^2) decreasing
  %}
    zones=[420,440,210,280,1,0,77,35,304,124;
              1,20,31,130,1,0,77,35,160,340;
              210,270,340,360,1,90,77,35,308,128];
 endfunction