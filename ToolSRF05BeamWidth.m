function [widthDeg,widthGrad] = ToolSRF05BeamWidth(distance)
  lawMatrix=[0,180;
            25,30;
            50,23;
            100,19;
            150,16;
            200,11;
            250,7;
            300,4;
            350,2;
            400,0];
  i=1;
  if (distance>lawMatrix(size(lawMatrix,1),1))
      widthDeg=0;
      widthGrad=0;
      return;
  endif
  while (i<size(lawMatrix,1))
    if(lawMatrix(i)>=distance)
      break
    endif
    i++;
  endwhile
  if i>size(lawMatrix,1)
    widthDeg=0;
  elseif (i==1)        
        widthDeg=0;
  else
    deltaDist=lawMatrix(i,1)-lawMatrix(i-1,1);
    deltaAngle=lawMatrix(i-1,2)-lawMatrix(i,2);
    ratioDist= (lawMatrix(i,1)-distance)/deltaDist;
    widthDeg=(lawMatrix(i,2)+(deltaAngle*ratioDist));
  endif
  widthGrad=widthDeg*pi()/180;
endfunction