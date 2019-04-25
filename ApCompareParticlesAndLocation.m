function [apRobot,robot,probability] = ApCompareParticlesAndLocation(apRobot,robot,radius,deltaHeading)

 % for some a cluster of particles look for all particles that are located around location(x,y,heading) inside a cercle of radius
 %  and return the sum of the probabilities

particles=apGet(apRobot,"particles");
location=apGet(apRobot,"location");
particlesNumber=size(particles,1);
probability=0;
radius2=radius^2;
for i=1:particlesNumber
  if (abs(ToolAngleDiff(location(3),particles(i,3))) <= deltaHeading)
    if (((location(1)-particles(i,1))^2+(location(2)-particles(i,2))^2)<=radius2)
      probability=probability+particles(i,4);
    endif
  endif
endfor
apRobot = setfield(apRobot,"locationProb",probability);
printf(mfilename);
printf(" Probalibity:%.1f%% . *** ",100*probability)
printf(ctime(time()));
save ("-mat4-binary","lastCompParticles.mat","particles")
endfunction
