function [] = CreateParticles(particlesNumber,cartoId,plotOn)
% cortoId to be developped
load carto1;
pw=0;
[x,y]=size(carto1);
particles=[];
while (i<=particlesNumber)
	px=randi(x);
	py=randi(y);
	po=randi(361)-1;

		if (QueryCartoAvailability(px,py,po,cartoId)==1)
			i=i+1;
		particles=[particles;[px,py,po,pw]];
		endif
end
save ("-mat4-binary","particles.mat","particles")
if (plotOn)
	figure();
	title ("particles");
	hold on;
	for i=1:particlesNumber
		plot(particles(i,1),particles(i,2))
	end
	hold off;
endif
endfunction
