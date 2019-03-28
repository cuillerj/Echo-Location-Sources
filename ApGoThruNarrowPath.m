function [apRobot,robot,retCode] = ApGoThruNarrowPath(apRobot,robot,pathNumber,entryPoint,pathHeading,pathDistance,farDistances,farther,forward)
    printf(mfilename);
    printf(" go thru path n°:%d  farther:%d forward:%d *** ",pathNumber,farther,forward);
    printf(ctime(time()));
    narrowPath=apGet(apRobot,"narrowPath");
    retCode=0;   
    location=apGet(apRobot,"location"); 
    startToEntryDistance=round(sqrt((location(1)-entryPoint(1))^2+(location(2)-entryPoint(2))^2)*1.5)  
    
    reqMove=0;
  %  [apRobot,robot,next,direct,forward] = ApComputeNextStepToTarget(apRobot,robot,0);
    if (forward==-9)
      retCode=-1;
      printf(mfilename);
      printf(" no way to go thru pass *** ",pathNumber);
      printf(ctime(time()));
      return 
    endif
   % requiredDirection=atan2(next,entryPoint)
   [number,type,pathOrientation,width,length,northHeadingF,northHeadingC] = GmapZones(apRobot,robot,entryPoint)
   if (farther)
        northHeading=northHeadingF
        farther=0;
     else
        northHeading=northHeadingC;
        farther=1;
   endif
   if (forward>=0)
        echoToGet=round(farDistances(1)*.7)
        forward=0;
     else
         echoToGet=round(farDistances(2)*.7)
         forward=1;
   endif
   [widthDeg,widthGrad] = ToolSRF05BeamWidth(width/2);
   length=round(length+width*sin(widthGrad))
   robot.MoveAcrossNarrowPass(pathDistance,width,length,startToEntryDistance,northHeading,reqMove,echoToGet);
  apRobot = setfield(apRobot,"waitFor",robot.moveAcrossPassEnded);
  debugOn=true;
  [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
  if (retCodeMove!=0 && apGet(apRobot,"realMode"))
    retCode=-1;
    return
  endif

  if(apGet(apRobot,"realMode"))
      robot.RequestNarrowPathMesurments;
      pause(3);
      while (robot.pendingNarrowPathMesurments)
          robot.RequestNarrowPathMesurments;
          pause(3);
      endwhile
      pathDistances=[robot.GetPathDistances(1),robot.GetPathDistances(2),robot.GetPathDistances(3),robot.GetPathDistances(4)];
      printf("distance to pass start:%dcm to pass entry:%dcm  path lengh:%dcm  after distance:%dcm \n",robot.GetPathDistances(1),robot.GetPathDistances(2),robot.GetPathDistances(3),robot.GetPathDistances(4));
      robot.RequestNarrowPathEchos;
      pause(3);
       while (robot.pendingNarrowPathEchos)
          robot.RequestNarrowPathEchos;
          pause(3);
      endwhile
        %robot.ValidHardPosition();
    else
      pathOrientationGrad=mod((pathOrientation*pi()/180)+pi()*forward+pi()*farther,2*pi())
      posX=(apGet(apRobot,"location")(1)+(startToEntryDistance/4+length+reqMove)*cos(pathOrientationGrad));
      posY=(apGet(apRobot,"location")(2)+(startToEntryDistance/4+length+reqMove)*sin(pathOrientationGrad));
      apRobot = setfield(apRobot,"location",[posX,posY,pathOrientation]);
      robot.SetPosX(posX);
      robot.SetPosY(posY);
      robot.SetHeading(pathOrientation);
      robot.UpdateHardRobotLocation();
  endif
   apRobot = setfield(apRobot,"locationProb",0.8);   
   apRobot = ApCreateParticlesForGoToTarget(apRobot,1);
    apRobot = setfield(apRobot,"locationProb",0.8);   
      % clear path enrty 
    if (size(narrowPath,1)<=2)
      apRobot = setfield(apRobot,"narrowPath",0)
    else
      apRobot = setfield(apRobot,"narrowPath",[0;[narrowPath(3,:)]])
    endif
    
endfunction