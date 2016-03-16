function [] = scenarioPFilterV2(plotOn)
% hypothese robot en 200 200 0
CreateParticles(1000,1,plotOn);
echoX=[200];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
	PlotEcho(echoX,echoY,'r')
endif

DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn)
echoX=[210];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
	PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn)

echoX=[220];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
	PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn) 

echoX=[230];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn)

echoX=[240];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn)

echoX=[250];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn) 

echoX=[260];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn)

echoX=[270];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,10,plotOn)

echoX=[280];
echoY=[200];
echoAngle=[0];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(0,20,plotOn)

echoX=[300];
echoY=[200];
echoAngle=[00]; 
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)

MoveParticles(90,50,plotOn)

echoX=[300];
echoY=[250];
echoAngle=[90];
echoProb=[98];
if (plotOn)
PlotEcho(echoX,echoY,'r')
endif
DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)

ResampleParticles(plotOn)
endfunction