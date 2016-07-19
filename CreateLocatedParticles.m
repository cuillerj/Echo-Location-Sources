function [particles] = CreateLocatedParticles(carto,posX,posY,heading,prob,particlesNumber,plotOn)
%{
create particles n=(prob/particlesNumber) located at (posX,posY,heading) with x y h precision
create particles n=(particlesNumber - n) located anywhere inide carto
%}
pw=0;
xPrecision=10;
yPrecision=10;
hPrecision=10;
particles=[];
[x,y]=size(carto);
lim=particlesNumber*prob/100;
i=0;
while (i<=lim)
	px=posX+randi(xPrecision)-xPrecision/2;
	py=posY+randi(yPrecision)-yPrecision/2;
	po=heading+randi(hPrecision)-hPrecision/2;
%		if (QueryCartoAvailability(carto,px,py,po*pi()/180,0)==1)
	i=i+1;
		particles=[particles;[px,py,po,pw]];
%		endif
end
i=0;
lim=(particlesNumber-lim)-1;
while (i<=lim)
	px=randi(x);
	py=randi(y);
	po=randi(361)-1;
	if (QueryCartoAvailability(carto,px,py,po*pi()/180,0)==1)
		i=i+1;
		particles=[particles;[px,py,po,pw]];
	endif
end

%save ("-mat4-binary","particles.mat","particles")
if (plotOn)
	figure();
	title ("created particles");
	hold on;
	for i=1:particlesNumber
		plot(particles(i,1),particles(i,2))
	end
	hold off;
endif
return
endfunction
