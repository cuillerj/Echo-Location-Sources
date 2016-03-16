function [] = MoveParticles(rotation,distance,plotOn)
load particles
noiseX=2;               % noise on x move
noiseY=noiseX; 			% noise on y move
noiseO=1;               % noise on rotation move
[x,y]=size(particles);
for i=1:x
	angle=(particles(i,3)+rotation)*pi/180;
	particles(i,1)=particles(i,1)+distance*cos(angle)+randi(noiseX)-noiseX/2;  % add noise
	particles(i,2)=particles(i,2)+distance*sin(angle)+randi(noiseY)-noiseY/2;
	particles(i,3)=mod(angle*180/pi,360)+(randi(noiseO)-noiseO/2);
endfor

save ("-mat4-binary","particles.mat","particles")
if (plotOn)
	figure();
	title ("particles");
	hold on;
	for i=1:x
		plot(particles(i,1),particles(i,2))
	end
	hold off;
endif
endfunction
