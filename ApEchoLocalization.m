 function [apRobot,robot,EchoLoc,automatonStateList] = ApEchoLocalization(apRobot,robot,flatLogRegMode,realMode,plotValue)
##
#
  %{
  main status
  %}
  initial=1;
  localizing=2;
  targeting=3;
  gotTarget=4;
  locked=5;
  lost=6;
  mStatus=["initial";"localizing";"targeting";"gotTarget";"locked";"lost"];
   %{
  localization status
  %} 
  notLocalized=1;
  localized=2;
  localisationLost=3;
  determining=4;
  lStatus=["notLocalized";"localized";"localisationLost";"determining"];
  %{
  action status
  %}
  atRest=1;
  NOrient=2;
  moving=3;
  scanned=4;
  aStatus=["atRest";"NOrient";"moving";"moving";"scanned"];

#

% stepLen=20;
%{
  preliminary step: create particles spread all over the space using CreateSpreadedParticles
  
  main steps
	align robot as during the learning phase based on compas
	scan 360 and use logistic regresion to guess weighted possible locations 
	take into account the compas orientation recorded during the learning phase to update the possible locations weights
	determine possibles locations particles taking into account updated weight

%}

 EchoLoc=[-1,-1,-1];
 maxRetry=5;
 more off;
 determine=5;
 determined=1;
 cpu1=cputime();
  if (!exist("flatLogRegMode"))  % flat logistic regression is default mode 
      flatLogRegMode=true;
  endif
  if (!exist("realMode"))  % simulation is default mode 
      realMode=0;
  endif
  if (!exist("plotValue"))  % real is default mode 
      plotValue=1;
  endif
 %simulationMode=1;  % to set simulation 1 on 0 off


  simulationMode=!realMode;
  printf(mfilename);
  printf(" real mode :%d *** ",realMode)
  printf(ctime(time()))
 % particlesNumber=2000;
  plotOn=true;                 % if true graphics will be provided (reduces performance)
  plotOff=false;               % no graphic
  noiseLevel=0.3;                 % coefficent (float) of noise of simualtion mode 0 no noise
  noiseRetCode=1;                 % boolean (0 move 100% completed 1 move can be incompleted)
  noiseRetValue=12;               % range of noised retcode in wich a random retcode is choosen
% [apRobot,robot] =ApInitApRobot(flatLogRegMode,realMode);
  apRobot = setfield(apRobot,"simulationMode",simulationMode);
  apRobot = setfield(apRobot,"realMode",realMode);
    %locationProbThresholdHigh=apGet(apRobot,"locProbThresholdHigh") 
  carto=apGet(apRobot,"carto");
  img=apGet(apRobot,"img");
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  rotationType=apGet(apRobot,"rotationType");  %  robot rotation 1: based on NO, 2: based on gyroscope, 3: based on encoder
  shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter")/10;
  %currentShiftNorthOrientation=apGet(apRobot,"currentShiftNorthOrientation");
  currentNorthOrientationReference=apGet(apRobot,"currentNorthOrientationReference");
  nbPred=10;                    % define the number of predictions that will be compute for each 360° scan
  located=false;
  alignParticles=true;
  newFigure=true;
  plotOn=true;
  checkLocationAvaibility=true;
  newFigure=true;
  traceDet=[];
  traceMove=[];
  traceEcho=[];
  forward=true;
  randomChoice=true;    
  newTheoriticalPositions=[184,184];  % set to the north oriented reference point
   if (plotValue>=1) % 
      figure(1);      % figure 1 fixed for moves
		  title ("moves");
		  hold on;
		  imshow(img,[]);  % insert map background
		  [a,b]=size(img);
		  axis([1,b,1,a],"on","xy");
      hold on;
   endif

  if (plotValue>=2)
      plotReq=true;
    else
      plotReq=false;
   endif

%   apRobot = setfield(apRobot,"location",[-1,-1,0]);
 %  apRobot = setfield(apRobot,"locationProb",0);
  % [apRobot]= ApCreateLocatedParticles(apRobot,particlesNumber,plotReq);  % create particles everywhere oriented as average during learning scan phase
  load ('spreadedParticles.mat');
  apRobot = setfield(apRobot,"particles",particles);
  %{
  if (realMode)
      robot.LaunchBatch();        % call java method to start all batchs
    else
      robot.LaunchSimu();        % call java method to start only simulator
  endif

  if (simulationMode)
      robot.noiseLevel=noiseLevel;                 % coefficent (float) of noise of simualtion mode 0 no noise
      robot.noiseRetCode=noiseRetCode;                 % boolean (0 move 100% completed 1 move can be incompleted)
      robot.noiseRetValue=noiseRetValue;               % range of noised retcode in wich a random retcode is choosen
      [apRobot,robot,randomLocation] = ApGetRandomLocation(apRobot,robot);
      printf(mfilename);
      printf(" Simulation ramdomly choosen physical location:(%d,%d,%d)  *** ",randomLocation(1),randomLocation(2),randomLocation(3));
      printf(ctime(time())); 
      printf(mfilename);
      printf(" Scan reference location :(%d,%d)  *** ",robot.simulatedHardX,robot.simulatedHardY);
      printf(ctime(time()));      
      printf(mfilename);
      printf(" Simulation noiseLevel:%f noiseRetCode:%d noiseRetValue:%d *** ",robot.noiseLevel,robot.noiseRetCode,robot.noiseRetValue);
      printf(ctime(time()));      
  endif
  if (realMode)
         robotStatus=robot.runningStatus;
         if (robotStatus<=0)
          printf(mfilename);
          printf(" wait for robot to be ready  *** ");
          printf(ctime(time()));
         endif
        while (robotStatus<=0)              % wait for robot to be ready
          robotStatus=robot.runningStatus;
          pause(2);
          printf(".");
        end
        pause(2);
         [apRobot,robot,rc] = ApInitRobotParameters(apRobot,robot);  % download parameters inside the robot
  endif
  %}
    pointLib=["theori:";"encoder:";"gyroTheo:";"BNOleft:";"BNORight:"];
    printf(mfilename);
    printf("align robot:%d  ",shiftEchoVsRotationCenter)
    printf(ctime(time()))
    if (plotValue>=1)
      plotReq=true;
     else
      plotReq=false;
    endif
 %   [apRobot,robot,retCode,action]=ApRobotNorthAlign(apRobot,robot,currentNorthOrientationReference,!alignParticles,plotReq);   % align robot such that it will be in the same orientation (average valu) as for learning phase
    count=0; 
  while (count<maxRetry && located==false) 
      automatonState=apGet(apRobot,"automatonState");
      automatonStateList=[apGet(apRobot,"automatonStateList");[automatonState]];
      apRobot = setfield(apRobot,"automatonStateList",automatonStateList);                    
      if(apGet(apRobot,"automatonRC")==0);  
        switch(automatonState)

          case([lost,notLocalized,atRest])
            printf(mfilename);
            printf(" lost state:(%d,%d,%d)  *** ",automatonState(1),automatonState(2),automatonState(3));
            printf(ctime(time()));     
            count=maxRetry;
            break    
          case {[localizing,notLocalized,atRest],[initial,notLocalized,atRest]}    
              [apRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,newTheoriticalPositions);
              robot.simulatedHardX=scanRef(1);
              robot.simulatedHardY=scanRef(2);
              currentNorthOrientationReference=scanRef(3);
              printf(mfilename);
              printf(" Scan reference location :(%d,%d) NO:%d *** ",robot.simulatedHardX,robot.simulatedHardY,currentNorthOrientationReference);
              printf(ctime(time()));
              [apRobot,robot,retCode,action]=ApRobotNorthAlign(apRobot,robot,currentNorthOrientationReference,alignParticles,plotReq); 

          case([localizing,notLocalized,NOrient])  
              %[apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithRotation(apRobot,robot,nbPred,0,plotOff);
              [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithTensorFlow(apRobot,robot);             
          case([localizing,notLocalized,scanned])
      
              [apRobot,robot,detX,detY,detH] = ApDetermineRobotLocationWithTfAndParticlesGaussian(apRobot,robot,posX,posY,posProb,plotReq,newFigure);
              if (realMode)
                 [apRobot,robot,retCode] = ApUpdateHardLocation(apRobot,robot,[detX,detY,detH],0);
                 if (retCode==-99)
                    printf(mfilename);
                    printf(" robot no longer communicate ");
                    printf(ctime(time())); 
                    return;                
                 endif
              endif  
              [apRobot,robot] = ApResampleParticles(apRobot,robot,plotReq,newFigure,checkLocationAvaibility);             
              if(count>0 && sqrt((detX(1)-newTheoriticalPositions(1))^2+(detY(1)-newTheoriticalPositions(2))^2)<30)
                  located=true;
                  EchoLoc=[detX(1),detY(1),detH(1)];
                  [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[determine,determined],1);
               else
                   [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[determine,!determined],1);          
               endif
              count++;       
          case{[localizing,determining,scanned],[locked,notLocalized,scanned]}     
                [apRobot,robot,rotationToDo,lenToDo] = ApEchoLocationDetermineNextMoveToScan(apRobot,robot,plotOff,!randomChoice);
                if (abs(lenToDo)>50)
                  lenToDo=50*(lenToDo/abs(lenToDo));
                endif
                if (abs(lenToDo)< apGet(apRobot,"minDistToBeDone"))
                      printf(mfilename);
                      printf(" no significative possible move:(%d,%d)",rotationToDo,lenToDo);
                      printf(ctime(time()));
                endif        
                [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOff);
   
          case([localizing,notLocalized,moving])  
                  [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOff);
                  if (retCodeMove!=robot.moveKoDueToNotEnoughSpace && retCodeMove!=robot.moveUnderLimitation)
                    if (realMode)
                      newTheoriticalPositions=round([robot.GetHardPosX(),robot.GetHardPosY()]);
                      printf(mfilename);
                      printf(" new Estimated Positions:(%d,%d)  *** ",newTheoriticalPositions(1),newTheoriticalPositions(2));
                      printf(ctime(time()));
                    else  
                      newTheoriticalPositions=round([robot.simulatedHardX,robot.simulatedHardY]+lenToDo*[cos(rotationToDo*pi()/180),sin(rotationToDo*pi()/180)]);
                      printf(mfilename);
                      printf(" new Theoritical Positions:(%d,%d)  *** ",newTheoriticalPositions(1),newTheoriticalPositions(2));
                      printf(ctime(time()));
                    endif
                   endif  

            
          case([locked,notLocalized,atRest])  
                  printf(mfilename);
                  printf(" locked state:(%d,%d,%d)  *** ",automatonState(1),automatonState(2),automatonState(3));
                  printf(ctime(time())); 
                  [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,plotOn);                    
    
   
          case([targeting,localized,NOrient])  
          
            located=true;
            return          
   
          otherwise
                  printf(mfilename);
                  printf(" unsuported state:(%d,%d,%d)  *** ",automatonState(1),automatonState(2),automatonState(3));
                  printf(ctime(time())); 
                  apRobot = setfield(apRobot,"automatonState",[lost,automatonState(2),automatonState(3)]);   
                  count=maxRetry;
          endswitch
         else
                 printf(mfilename);
                 printf(" unsuported move for:(%d,%d,%d)  *** ",automatonState(1),automatonState(2),automatonState(3));
                 printf(ctime(time())); 
                 automatonState=apGet(apRobot,"automatonState");
                 apRobot = setfield(apRobot,"automatonState",[lost,automatonState(2),automatonState(3)]);        
                 count=maxRetry;        
         endif
      endwhile
        
        if (realMode)
          if (EchoLoc(1)==-1)
            robot.Horn(5);  % horn x seconds
            else
              for i=1:3
                  robot.Horn(1);  % horn x seconds
                  pause(1);
              end
          endif       
      endif
endfunction