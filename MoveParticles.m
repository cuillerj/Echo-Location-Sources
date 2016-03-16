function [] = MoveParticles(rotation,distance,plotOn)
load particles
[x,y]=size(particles);
for i=1:x
	angle=(particles(i,3)+rotation)*pi/180;
	particles(i,1)=particles(i,1)+distance*cos(angle)+randi(4)-2;
	particles(i,2)=particles(i,2)+distance*sin(angle)+randi(4)-2;
	particles(i,3)=mod(angle*180/pi,360)+(randi(2)-1)/20;
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
