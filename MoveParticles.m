function [particles] = MoveParticles(rotation,distance,plotOn,particles)
%load particles
noiseX=2;               % noise on x move
noiseY=noiseX; 			% noise on y move
noiseO=1;               % noise on rotation move
noiseDist=150;
noiseRot=300;
%distance=distance+distance*(randi(noiseDist)-noiseDist/2)/1000 % add noise
%rotation=rotation+rotation*(randi(noiseRot)-noiseRot/2)/1000;
[x,y]=size(particles);
for i=1:x
	dist=distance+distance*(randi(noiseDist)-noiseDist/2)/1000; % add noise
	rot=rotation+rotation*(randi(noiseRot)-noiseRot/2)/1000;
	angle=(particles(i,3)+rot)*pi/180;
	particles(i,1)=particles(i,1)+dist*cos(angle);  
	particles(i,2)=particles(i,2)+dist*sin(angle);
	particles(i,3)=mod(angle*180/pi,360);
endfor

%save ("-mat4-binary","particles.mat","particles")
if (plotOn)
	figure();
	title ("particles moved");
	hold on;
	for i=1:x
		plot(particles(i,1),particles(i,2))
	end
	hold off;
endif
endfunction
