function [particles] = MoveParticles(rotation,distance,img,plotOn,particles,shiftEchoVsRotationCenter)
rotation=mod(rotation+360,360);
%load particles
%noiseX=2;               % noise on x move
%noiseY=noiseX; 			% noise on y move
%noiseO=1;               % noise on rotation move
%noiseDist=100;           % distance can be increase or decrease up to noiseDist/1000 in pourcentage
sigmaDist=2*distance/100;
%sigmaDist=0;
sigmaRot=1; %1
sigmaHeading=0.5; %0.5
%noiseRot=6000;           % when rotation required rotation can be increase or decrease up to noiseRot/1000 degres
%noiseStraight=3000;       % when straight move 
%distance=distance+distance*(randi(noiseDist)-noiseDist/2)/1000 % add noise
%rotation=rotation+rotation*(randi(noiseRot)-noiseRot/2)/1000;
[x,y]=size(particles);
for i=1:x
%	dist=distance+distance*(randi(noiseDist)*sign(randi([-1,1]))/1000); % add noise
	dist=normrnd(distance,sigmaDist); % add noise
	if (rotation==0)
%		rot=(randi(noiseStraight)*(sign(randi([-1,1]))/1000));	
%		rot=
		rot=normrnd(0,sigmaHeading);
	else
		rot=normrnd(rotation,sigmaRot);
%		rot=rotation+(randi(noiseRot)*(sign(randi([-1,1]))/1000));
	endif
  currentH=particles(i,3)*pi/180;
	angle=currentH+rot*pi/180;
	particles(i,1)=particles(i,1)+dist*cos(angle)+shiftEchoVsRotationCenter*cos(angle)-shiftEchoVsRotationCenter*cos(currentH);  
	particles(i,2)=particles(i,2)+dist*sin(angle)+shiftEchoVsRotationCenter*sin(angle)-shiftEchoVsRotationCenter*sin(currentH);
	particles(i,3)=mod(angle*180/pi,360);
endfor

%save ("-mat4-binary","particles.mat","particles")
if (plotOn)
	figure();
	title ("moved particles");
	hold on;
	imshow(img,[])
	[a,b]=size(img);
	axis([1,b,1,a],"on","xy");
	for i=1:x
		plot(particles(i,1),particles(i,2))
	end
	hold off;
endif
endfunction
