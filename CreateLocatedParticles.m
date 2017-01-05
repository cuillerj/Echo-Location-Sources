function [particles] = CreateLocatedParticles(carto,img,posX,posY,heading,prob,particlesNumber,plotOn)
%{
create particles n=(prob/particlesNumber) located at (posX,posY,heading) with x y h precision
create particles n=(particlesNumber - n) located anywhere inide carto
if posX <0 or posY <0 particles are created anywhere
if heading <0 particles are created in any orientation
if prob > 0 some particles are created according to posX posY and heading 
if posX and posY <0 and heading > 0 prob will not be used
%}
pw=0;
xPrecision=10;
yPrecision=10;
hPrecision=10;
sigmaDist=0.2;  %2
sigmaHeading=0.0;
particles=[];
[x,y]=size(carto);
lim=particlesNumber*prob/100;
if (posX<0 || posY<0)
	lim=0
endif
i=0;
while (i<=lim)             % create some particles according to posX posY and heading
%	px=posX+randi(xPrecision)-(xPrecision+1)/2;
%	py=posY+randi(yPrecision)-(yPrecision+1)/2;
%	po=heading+randi(hPrecision)-(hPrecision+1)/2;
	px=normrnd(posX,sigmaDist);
	py=normrnd(posY,sigmaDist);
	po=normrnd(heading,sigmaHeading);
%		if (QueryCartoAvailability(carto,px,py,po*pi()/180,0)==1)
	i=i+1;
		particles=[particles;[px,py,po,pw]];
%		endif
end
i=0;
if (lim==0 && heading >=0)   
	lim=(particlesNumber-lim)-1;
	while (i<=lim)        % create some particles anywhere in any direction
	px=randi(x);
	py=randi(y);
	po=heading+randi((100+hPrecision-prob))-(100+hPrecision+1-prob)/2;
	if (QueryCartoAvailability(carto,px,py,po*pi()/180,0)==1)
		i=i+1;
		particles=[particles;[px,py,po,pw]];
	endif
	end
lim=0;
else
	lim=(particlesNumber-lim)-1;
endif
while (i<=lim)        % create some particles anywhere in any direction
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
	imshow(img,[])
	[a,b]=size(img);
	axis([1,b,1,a],"on","xy");
	for i=1:particlesNumber
		plot(particles(i,1),particles(i,2))
	end
	hold off;
endif
return
endfunction
