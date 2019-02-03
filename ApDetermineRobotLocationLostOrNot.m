function [located,consistant,distance2loc,deltaH,bestLocation] = ApDetermineRobotLocationLostOrNot(apRobot,savLoc,savProb,TfLocation,TfProb)
  %{
    
  %}
   located = false;
   consistant = false;     
   determineThreshold=apGet(apRobot,"determinationThreshold");
   DetTheoDistanceMaxi=determineThreshold(1);
   DetTfDistanceRef=determineThreshold(2);
   DetTfDistanceMaxi=determineThreshold(3);
   ParticlesProbMin=determineThreshold(4);
   ParticlesProbRef=determineThreshold(5);
   TfHighLevel=determineThreshold(6);
   TfHighestLevel=determineThreshold(7);
   ParticlesProbMini=determineThreshold(8);
   TfLowLevel=determineThreshold(9);
   TfCertainty=determineThreshold(10);
   headingMargin=apGet(apRobot,"headingMargin");                         % 
   radiusMargin=apGet(apRobot,"radiusMargin");                        %
   distance2loc=sqrt((apGet(apRobot,"location")(1)-savLoc(1))^2+(apGet(apRobot,"location")(2)-savLoc(2))^2);
   distance2Tf=sqrt((apGet(apRobot,"location")(1)-TfLocation(1))^2+(apGet(apRobot,"location")(2)-TfLocation(2))^2);
   deltaH=ToolAngleDiff(apGet(apRobot,"location")(3),savLoc(3));
   bestLocation=[-1,-1,-1];
   if (TfProb>=TfCertainty && distance2Tf >radiusMargin)
       located = true;
       consistant = false;
       bestLocation=[TfLocation(1),TfLocation(2),apGet(apRobot,"location")(3)];
       return;
   endif   
   if (TfProb>=TfHighLevel && distance2Tf <=radiusMargin)
       bestLocation=apGet(apRobot,"location");
       located = true;
   elseif (savProb>=apGet(apRobot,"locationProb") && distance2loc<=radiusMargin && abs(deltaH)<=headingMargin)
       bestLocation=savLoc;
       located = true;      
   endif
  if (distance2loc<=radiusMargin && abs(deltaH)<=headingMargin)
       consistant = true;      
   endif
   printf(mfilename);
   printf(" Located:%d  consistant:%d ***  ",located,consistant);
   printf(ctime(time()));
   return;
endfunction