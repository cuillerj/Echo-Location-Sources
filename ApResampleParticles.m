function [apRobot,robot] = ApResampleParticles(apRobot,robot,plotOn,newFigure,checkLocationAvaibility)
%load particles
  plotRatio=5; # 1/plotRatio point will be plotted
  fast=true;
  if (!exist("newFigure"))  % simulation is default mode 
      newFigure=false;
  endif
    if (!exist("checkLocationAvaibility"))  % simulation is default mode 
      checkLocationAvaibility=false;
  endif
  printf(mfilename);
  printf("   ***   ");
  printf(ctime(time()))
  particles=apGet(apRobot,"particles");  
  carto=apGet(apRobot,"carto");
  [cartoX,cartoY]=size(carto);
  cartoMaxAvailableValue=apGet(apRobot,"cartoMaxAvailableValue");  
  img=apGet(apRobot,"img");    
  [nbPart,y]=size(particles);
  radianUnit=1;
  debugOff=0;
  if (checkLocationAvaibility)

    for i=1:nbPart
      
    %      if (ApQueryCartoAvailability(apRobot,[particles(i,1),particles(i,2),particles(i,3)*pi()/180],radianUnit,debugOff,!fast)==0)
     %     i=i+1;
       x=round(particles(i,1));
       y=round(particles(i,2));
       if (x <= 0 || x > cartoX)
           particles(i,4)=0;
       endif
       if (y <= 0 || y > cartoY)
           particles(i,4)=0;
       endif
       if (particles(i,4)!=0 && carto(x,y)>cartoMaxAvailableValue) 
           particles(i,4)=0;
       endif
    end
    %}
    
  endif
  omegaMax=max(particles(:,4));
  %idx=[1:nbPart];
  idx=randi(nbPart-1);
  pick=[];
  beta=0;
  for i=0:nbPart-1
    beta=beta+rand(1)*(2*omegaMax);
    while particles(idx+1,4)<beta
      beta=beta-particles(idx+1,4);
      idx=mod(idx+1,nbPart);
    end
    pick=[pick,[idx+1]];
  endfor
  %pick
  newParticles=[];
  for i=1:nbPart
    newParticles=[newParticles;[particles(pick(i),1:3)]];
  endfor
  z=ones(nbPart,1)*1/nbPart;
  particles=[newParticles,z];
  apRobot = setfield(apRobot,"particles",particles);
  %save ("-mat4-binary","particles.mat","particles");
  x=nbPart;
  if (plotOn)
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
    if (newFigure)
      figure()
      hold on;
      imshow(img,[]);  % insert map background
      [a,b]=size(img);
      axis([1,b,1,a],"on","xy");
      hold on;
     else
      figure(3);
    endif
    title ("resampled particles ");
    hold on;
  %	imshow(img,[])
  %	[a,b]=size(img);
  %	axis([1,b,1,a],"on","xy");
    for i=1:(round(x/plotRatio)-1)
      plot(particles(plotRatio*i,1)+shitfCartoX,particles(plotRatio*i,2)+shitfCartoY)
    end
    hold off;
  endif
endfunction

