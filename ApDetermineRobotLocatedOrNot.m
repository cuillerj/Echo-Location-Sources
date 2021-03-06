function [located,consistant] = ApDetermineRobotLocatedOrNot(apRobot,traceLoc)
   located=false;
   consistant=true;
   step=size(traceLoc,1);
   determineThreshold=apGet(apRobot,"determinationThreshold");
   DetTfDistance=sqrt((traceLoc(step,3)-traceLoc(step,12))^2+(traceLoc(step,4)-traceLoc(step,13))^2);
   TheoRobotDistance=traceLoc(step,11);
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
     elseif (TfProb>=TfHighLevel && DetTfDistance<=DetTfDistanceRef && TheoRobotDistance<=DetTfDistanceRef)
       located = true;     
     elseif (TfProb>=TfHighLevel && ParticlesProb>=ParticlesProbRef)
       located = true; 
     elseif (ParticlesProb>ParticlesProbMin &&  TheoRobotDistance<DetTheoDistanceMaxi &&  DetTfDistance <DetTfDistanceMaxi)
       located = true; 
     endif
     %if (((DetTfDistance>DetTfDistanceMaxi)|| ParticlesProb < ParticlesProbMini) && TfProb>=TfLowLevel)
     if ((DetTfDistance>DetTfDistanceMaxi) && TfProb>=TfLowLevel)     
          printf(mfilename);
          printf(" consitancy issue between particles filter and tensor flow: (%fcm,%f%%) *** ",DetTfDistance,ParticlesProb*100);
          printf(ctime(time()))
       consistant=false;
      elseif (ParticlesProb <= ParticlesProbMin)
          consistant=false;
     endif
   endif
   printf(mfilename);
   printf(" located:%d consistant:%d ***  ",located,consistant);
   printf(ctime(time()))
   checked=false;
   determinationLocatedOrNot=[step,traceLoc(step,:),located,consistant,checked,apGet(apRobot,"realMode")];
   dlmwrite("determinationLocatedOrNot.txt",determinationLocatedOrNot,'delimiter',',','-append');
endfunction