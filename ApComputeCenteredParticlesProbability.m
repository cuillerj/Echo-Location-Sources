function [probability] = ApComputeCenteredParticlesProbability(particles,location,radius,headingMargin)
  %{
  for some a cluster of particles look for all particles that are located around location(x,y,heading) inside a cercle of radius
    and return the sum of the probabilities
  %}
  particlesNumber=size(particles,1);
  radius2=radius^2;
  probability=0;
  for i=1:particlesNumber
    if (mod(ToolAngleDiff(location(3),particles(i,3))-180,360) <= mod(headingMargin-180,360))
      if (((location(1)-particles(i,1))^2+(location(2)-particles(i,2))^2)<=radius2)
        probability=probability+particles(i,4);
      endif
    endif
 endfor
 printf(mfilename);
 printf(" Probalibity:%d%% . *** ",100*probability)
 printf(ctime(time()));
endfunction