function [] = scenarioPFilterV3(nbPart,deltaO,deltaD,plotOn)
% hypothese robot en 200 200 0
CreateParticles(nbPart,1,plotOn);
x=200;
y=200;
ard=0;
	alpha=(mod(deltaO,360)*pi)/180;
for i=1:10
	echoX=[x];
	echoY=[y];
	if (plotOn)
		PlotEcho(echoX,echoY)
	endif
	echoAngle=0;
	echoProb=[90];
	DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)
	ResampleParticles(plotOn);
	MoveParticles(deltaO,deltaD,plotOn);
	ard=mod(ard+alpha,2*pi)+(randi(2)-1)/20 % add noise
	x=x+deltaD*cos(ard)+randi(4)-2;
	y=y+deltaD*sin(ard)+randi(4)-2;
endfor
	DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)
	ResampleParticles(plotOn)
	x
	y
	ard*180/pi
	if (plotOn)
		PlotEcho(echoX,echoY)
	endif
	% angle to developp
endfunction