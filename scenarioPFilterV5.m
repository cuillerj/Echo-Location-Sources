function [] = scenarioPFilterV5(plotOn)
% hypothese robot en 200 200 0
particles=CreateParticles(5000,1,0);
echoX=[153,153,119,187,153,187,221,187,153,221];
echoY=[187,153,221,221,221,187,187,153,153,221];
echoAngle=[1,359,3,5,19,0,2,356,359,6];
echoProb=[60,18,6,6,4,60,18,6,6,4];
move=[0,0,0,0,33,0,0,0,0];
rot=[0,0,0,0,0,0,0,0,0];

for i=1:size(move,2)
	[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX(i),echoY(i),echoProb(i),plotOn,particles);
	[detX,detY,detH]
	particles=ResampleParticles(plotOn,particles);
	particles=MoveParticles(rot(i),move(i),plotOn,particles);
endfor
	[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX(i+1),echoY(i+1),echoProb(i+1),plotOn,particles);
	[detX,detY,detH]
	particles=ResampleParticles(1,particles);

save ("-mat4-binary","particles.mat","particles")

endfunction