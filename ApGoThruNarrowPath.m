function [apRobot,robot,retCode] = ApGoThruNarrowPath(apRobot,robot,pathNumber,entryPoint,pathHeading,pathDistance,farDistances,farther,forward)
    printf(mfilename);
    printf(" go thru path n°:%d  farther:%d forward:%d *** ",pathNumber,farther,forward);
    printf(ctime(time()));
    narrowPath=apGet(apRobot,"narrowPath");
    retCode=0;   
    location=apGet(apRobot,"location"); 
    startToEntryDistance=80;    
    reqMove=50;
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
        echoToGet=farDistances(1);
        forward=0;
     else
         echoToGet=farDistances(2);
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
  robot.RequestNarrowPathMesurments;
  pause(2);
  robot.RequestNarrowPathEchos;
  if(apGet(apRobot,"realMode"))
        robot.ValidHardPosition();
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