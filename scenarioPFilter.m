function [] = scenarioPFilter()
% hypothese robot en 200 200 0
CreateParticles(1000,1);
echoX=[100,200,300];
echoY=[100,200,300];
echoAngle=[90,0,180];
echoProb=[1,98,1];
PlotEcho(echoX,echoY,'r')

DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10)

echoX=[200,210,310];
echoY=[100,200,220];
echoAngle=[45,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10)

echoX=[250,220,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10) 

echoX=[250,230,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10)

echoX=[250,240,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10)

echoX=[250,250,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10) 

echoX=[250,260,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10)

echoX=[250,270,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,10)

echoX=[250,280,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,20)

echoX=[250,300,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles

MoveParticles(0,20)

echoX=[250,320,320];
echoY=[100,200,220];
echoAngle=[0,0,180];
echoProb=[30,90,10];
PlotEcho(echoX,echoY,'r')
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoAngle,echoProb)

ResampleParticles
endfunction