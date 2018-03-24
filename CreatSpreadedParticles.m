function [apRobot,robot] = CreateSpreadedParticles(plotOn)
   [apRobot,robot] =ApInitApRobot(1,0);
   apRobot = setfield(apRobot,"location",[-1,-1,0];
   apRobot = setfield(apRobot,"locProb",0);
   if (!exist("plotOn"))  % simulation is default mode 
       plotOn=0;
    endif
   partNumber=input("particles number to be created (0 to stop): ");
   if (partNumber=0)
     return
   endif
   [apRobot] = ApCreateLocatedParticles(apRobot,partNumber,plotOn);
   save ("-mat","spreadedParticles.mat","p");
  return
   