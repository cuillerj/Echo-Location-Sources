function [located,consistant] = ApDetermineRobotLocatedOrNot(apRobot,traceLoc)
   printf(mfilename);
   printf(" ***  ");
   printf(ctime(time()))
   located=false;
   consistant=true;
   step=size(traceLoc,1);
   determineThreshold=apGet(apRobot,"determinationThreshold");
   DetTfDistance=sqrt((traceLoc(step,3)-traceLoc(step,12))^2+(traceLoc(step,4)-traceLoc(step,13))^2);
   DetTheoDistance=traceLoc(step,11);
   ParticlesProb=traceLoc(step,6);
   TfProb=traceLoc(step,14);
   DetTheoDistanceMaxi=determineThreshold(1);
   DetTfDistanceRef=determineThreshold(2);
   DetTfDistanceMaxi=determineThreshold(3);
   ParticlesProbMin=determineThreshold(4);
   ParticlesProbRef=determineThreshold(5);
   TfHighLevel=determineThreshold(6);
   TfHighestLevel=determineThreshold(7);
   ParticlesProbMini=determineThreshold(8);
   TfLowLevel=determineThreshold(9);
   
   if (step==1) 
      determinationLocatedOrNot=[step,traceLoc(step,:),located,consistant,0,apGet(apRobot,"realMode")];
      dlmwrite("determinationLocatedOrNot.txt",determinationLocatedOrNot,'delimiter',',','-append');
     return;
   else
     if (TfProb>=TfHighestLevel)
       located = true;
     elseif (TfProb>=TfHighLevel && DetTfDistance<=DetTfDistanceRef)
       located = true;     
     elseif (TfProb>=TfHighLevel && ParticlesProb>=ParticlesProbRef)
       located = true; 
     elseif (ParticlesProb>ParticlesProbMin &&  DetTheoDistance<DetTheoDistanceMaxi &&  DetTfDistance <DetTfDistanceMaxi)
       located = true; 
     endif
     if ((DetTfDistance>DetTfDistanceMaxi || ParticlesProb < ParticlesProbMini) && TfProb>=TfLowLevel)
          printf(mfilename);
          printf(" consitancy issue between particles filter ans tensor flow: (%fcm,%f%%) *** ",DetTfDistance,ParticlesProb*100);
          printf(ctime(time()))
       consistant=false;
     endif
   endif
   checked=false;
   determinationLocatedOrNot=[step,traceLoc(step,:),located,consistant,checked,apGet(apRobot,"realMode")];
   dlmwrite("determinationLocatedOrNot.txt",determinationLocatedOrNot,'delimiter',',','-append');
endfunction