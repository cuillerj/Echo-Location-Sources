function [apRobot,robot] = CreateSpreadedParticles(plotOn)
  %{
  create spreaded particles oriented as scanOrientation
  %}
  scanOrientation=0;
   [apRobot,robot] =ApInitApRobot(1,0);
   apRobot = setfield(apRobot,"location",[-1,-1,scanOrientation]);
   apRobot = setfield(apRobot,"locProb",100);
   if (!exist("plotOn"))  % simulation is default mode 
       plotOn=0;
    endif
   partNumber=input(" enter particles number to be created (0 to stop): ");
   if (partNumber==0)
     return
   endif
  [apRobot] = ApCreateLocatedParticles(apRobot,partNumber,plotOn);
   particles=apGet(apRobot,"particles");
   size(particles)
   save ("-mat4-binary","spreadedParticles.mat","particles");
  return
   