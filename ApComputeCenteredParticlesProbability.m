function [prob] = ApComputeCenteredParticlesProbability(particles,location,radius,headingMargin)
  %{
  for some a cluster of particles look for all particles that are located around location(x,y,heading) inside a cercle of radius
    and return the sum of the probabilities
  %}
  particlesNumber=size(particles,1);
  sqRadius=radius^2;
  prob=0;
  for i=1:particlesNumber
    if ((particles(i,1)-location(1))^2+(particles(i,2)-location(2))^2 <= sqRadius)
      delta = ToolAngleDiff(particles(i,3),location(3)); 
      if (delta<=headingMargin)
        prob=prob+particles(i,4);
      endif
    endif
 endfor
endfunction