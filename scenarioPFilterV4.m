function [] = scenarioPFilterV4(plotOn)
% hypothese robot en 200 200 0
particles=CreateParticles(1000,1,0);
echoX=[153];
echoY=[187];
echoAngle=[1];
echoProb=[60];


[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);

particles=MoveParticles(0,0,0,particles);
echoX=[153];
echoY=[153];
echoAngle=[1];
echoProb=[18];

[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);

particles=MoveParticles(0,33,0,particles);

echoX=[187];
echoY=[187];
echoAngle=[1];
echoProb=[60];

[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);

particles=MoveParticles(0,0,0,particles);


echoX=[221];
echoY=[187];
echoAngle=[1];
echoProb=[18];

[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);

particles=MoveParticles(0,33,0,particles);

echoX=[255];
echoY=[187];
echoAngle=[3];
echoProb=[60];

[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);
particles=MoveParticles(0,0,0,particles);

echoX=[153];
echoY=[51];
echoAngle=[3];
echoProb=[18];

[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);

particles=MoveParticles(0,33,0,particles);

echoX=[221];
echoY=[153];
echoAngle=[7];
echoProb=[60];

[detX,detY,detH,particles]=DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles);
[detX,detY,detH]
particles=ResampleParticles(plotOn,particles);
particles=MoveParticles(0,0,0,particles);

echoX=[225];
echoY=[221];
echoAngle=[3];
echoProb=[18];

save ("-mat4-binary","particles.mat","particles")

endfunction