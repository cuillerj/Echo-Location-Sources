function [particles] = ResampleParticles(img,plotOn,particles)
%load particles
[nbPart,y]=size(particles);
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
z=zeros(nbPart,1);
particles=[newParticles,z];
%save ("-mat4-binary","particles.mat","particles");
if (plotOn)
	figure()
	title ("resampled particles ");
	hold on;
	imshow(img,[])
	[a,b]=size(img);
	for i=1:nbPart
		plot(particles(i,1),a+1-particles(i,2),'m');
	end
	hold off;
endif
endfunction

