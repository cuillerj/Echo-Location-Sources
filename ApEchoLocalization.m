 function [apRobot,robot,EchoLoc,traceLoc] = ApEchoLocalization(apRobot,robot,flatLogRegMode,realMode,plotValue,loopId)
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
  aStatus=["atRest";"NOrient";"moving";"scanned"];

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
 maxRetry=8;
 determinedDistanceThreshold=30;    # expressed in cm
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
  if(!realMode)
      printf(mfilename);
      printf(" simulation move noise:%d  scan noise:%d  retcode noise:%d retcode value;%d *** ",robot.noiseLevel,robot.scanNoiseLevel,robot.noiseRetCode,robot.noiseRetValue);
      printf(ctime(time()))
  endif
  simulationMode=!realMode;
  printf(mfilename);
  printf(" real mode :%d *** ",realMode)
  printf(ctime(time()))
 % particlesNumber=2000;
  plotOn=true;                 % if true graphics will be provided (reduces performance)
  plotOff=false;               % no graphic
  #noiseLevel=0.3;                 % coefficent (float) of noise of simualtion mode 0 no noise
  #scanNoiseLevel=5;
  #noiseRetCode=1;                 % boolean (0 move 100% completed 1 move can be incompleted)
  #noiseRetValue=12;               % range of noised retcode in wich a random retcode is choosen
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
  consistant=true;
  alignParticles=true;
  newFigure=true;
  plotOn=true;
  checkLocationAvaibility=true;
 % traceDet=[];
  traceMove=[];
  histDeltaDist=[0];
  if (!exist("traceLoc"))
    traceLoc=[];
  endif
  forward=true;
  randomChoice=false;
  if(apGet(apRobot,"simulationMode"))         
    randomPosition = ApGetRadomPosition(apRobot);
    newTheoriticalPositions=randomPosition(1:2);
    printf(mfilename);
    printf(" randomly choosen location: (%d,%d) *** ",newTheoriticalPositions(1),newTheoriticalPositions(2))
    printf(ctime(time()))
  else
    newTheoriticalPositions=[184,184];  % 
  endif
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
   load ('5000particlesSpreadedXoriented.mat');
   apRobot = setfield(apRobot,"particles",particles);
    pointLib=["theori:";"encoder:";"gyroTheo:";"BNOleft:";"BNORight:"];
    printf(mfilename);
    printf("align robot:%d  ",shiftEchoVsRotationCenter)
    printf(ctime(time()))
    if (plotValue>=1)
      plotReq=true;
     else
      plotReq=false;
    endif
    if (plotValue>=2)
      plotReqL2=true;
     else
      plotReqL2=false;
    endif
 %   [apRobot,robot,retCode,action]=ApRobotNorthAlign(apRobot,robot,currentNorthOrientationReference,!alignParticles,plotReq);   % align robot such that it will be in the same orientation (average valu) as for learning phase
    count=1;

    countDet=1;
    lastMesurmentReliable=false; 
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
            if (count==1)
              [pRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,apGet(apRobot,"northOrientationReferencePoint"));   
              currentNorthOrientationReference=scanRef(3);   
              if(apGet(apRobot,"simulationMode"))          
                robot.simulatedHardX=newTheoriticalPositions(1);
                robot.simulatedHardY=newTheoriticalPositions(2); 
              else           
                robot.simulatedHardX=scanRef(1);
                robot.simulatedHardY=scanRef(2);      
              endif
            else
                [pRobot,robot,scanRef,distance] = ApGetClosestScanReference(apRobot,robot,newTheoriticalPositions);             
                robot.simulatedHardX=scanRef(1);
                robot.simulatedHardY=scanRef(2);
                currentNorthOrientationReference=scanRef(3);  
            endif
  
              printf(mfilename);
              printf(" Scan reference location :(%d,%d) NO:%d *** ",robot.simulatedHardX,robot.simulatedHardY,currentNorthOrientationReference);
              printf(ctime(time()));             
              [apRobot,robot,retCode,action]=ApRobotNorthAlign(apRobot,robot,currentNorthOrientationReference,alignParticles,(plotValue>=2)); 
              if (action=="inMove" || action=="stop..")
                  printf(mfilename);
                  printf(" stop required after NorthAlign : recode %d, action (%d,%d)  *** ",retCode,action(1),action(2));
                  printf(ctime(time())); 
                  apRobot = setfield(apRobot,"automatonState",[lost,automatonState(2),automatonState(3)]);   
                  count=maxRetry;
              endif
          case([localizing,notLocalized,NOrient])  
              %[apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithRotation(apRobot,robot,nbPred,0,plotOff);
              [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithTensorFlow(apRobot,robot,plotReqL2,0);             
          case([localizing,notLocalized,scanned])
              traceLoc=[traceLoc;[zeros(1,20)]];
              traceLoc(size(traceLoc)(1),1:2)=[loopId,countDet];    
              [apRobot,robot,detX,detY,detH,prob,figureNumber] = ApDetermineRobotLocationWithTfAndParticlesGaussian(apRobot,robot,posX,posY,posProb,(plotValue>=1),newFigure);
              if((apGet(apRobot,"realMode") && plotValue>=1 && count>1) || (apGet(apRobot,"simulationMode") && plotValue>=1) )
                figure(figureNumber);
                plot(newTheoriticalPositions(1)+shitfCartoX,newTheoriticalPositions(2)+shitfCartoY,"color","r","+","markersize",15)
                hold off;
              endif
              probCum = ApComputeCenteredParticlesProbability(apGet(apRobot,"particles"),[detX,detY,detH],apGet(apRobot,"distanceMargin"),apGet(apRobot,"headingMargin"));
              printf(mfilename);
              printf(" determined location (%d,%d):particles (%d,%d,%d %.1f%%) TensorFlow(%d,%d %.1f%%) *** ",loopId,count,detX,detY,detH,probCum*100,posX(1),posY(1),posProb(1)*100);
              printf(ctime(time())); 
              traceLoc(size(traceLoc)(1),3:6)=[detX,detY,detH,probCum];
              traceLoc(size(traceLoc)(1),12:14)=[posX(1),posY(1),posProb(1)];
              countDet++;
              if (realMode)
                 [apRobot,robot,retCode] = ApUpdateHardLocation(apRobot,robot,[detX,detY,detH],prob);
                 if (retCode==-99)
                    printf(mfilename);
                    printf(" robot no longer communicate ");
                    printf(ctime(time())); 
                    return;                
                 endif
              else
                  printf(mfilename);
                  printf(" distance with theoritical poosition (%d,%d): %d cm ***",newTheoriticalPositions(1),newTheoriticalPositions(2),sqrt((newTheoriticalPositions(1)-detX)^2+(newTheoriticalPositions(2)-detY)^2));
                  printf(ctime(time())); 
              endif

              localizationFound=true;
              deltaDist=sqrt((detX-newTheoriticalPositions(1))^2+(detY-newTheoriticalPositions(2))^2);
              histDeltaDist(countDet)=deltaDist;
              if (countDet>2)
                traceLoc(size(traceLoc)(1),9:11)=[newTheoriticalPositions(1),newTheoriticalPositions(2),deltaDist];
              endif
        #      if(count>0 && lastMesurmentReliable && deltaDist<determinedDistanceThreshold && prevDeltaDist < determinedDistanceThreshold*2)
              [located,consistant]= ApDetermineRobotLocatedOrNot(apRobot,traceLoc);
              if (located)
                   EchoLoc=[detX,detY,detH];
                  [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[determine,localizationFound],1);           
              %{
              if(count>2 && (mod(detH,360)<=15 || mod(detH,360)>=345) && lastMesurmentReliable && histDeltaDist(countDet)<determinedDistanceThreshold && histDeltaDist(countDet-1)<determinedDistanceThreshold*2 )
                  located=true;
                  EchoLoc=[detX,detY,detH];
                  [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[determine,localizationFound],1);
                  %}
               else
                   [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[determine,!localizationFound],1);
                   [apRobot,robot] = ApResampleParticles(apRobot,robot,plotReq,newFigure,checkLocationAvaibility);
                 %  probCum = ApComputeCenteredParticlesProbability(apGet(apRobot,"particles"),[detX,detY,detH],apGet(apRobot,"distanceMargin"),apGet(apRobot,"headingMargin"))
                    prevDeltaDist=deltaDist;               
               endif
              if(count>4 && mean(histDeltaDist)>determinedDistanceThreshold*3);
                count=maxRetry;
              elseif (!consistant)
                count=maxRetry;                
              else
                count++;
              endif       
          case{[localizing,determining,scanned],[locked,notLocalized,scanned]} 
                if (automatonState(1)==locked)
                  randomChoice=true;
                else
                  randomChoice=false;
                endif
                [apRobot,robot,rotationToDo,lenToDo] = ApEchoLocationDetermineNextMoveToScan(apRobot,robot,(plotValue>=3),randomChoice);
                if (abs(lenToDo)>40)
                  lenToDo=40*(lenToDo/abs(lenToDo))+round(rand*20-10);
                endif
                if (abs(lenToDo)< apGet(apRobot,"minDistToBeDone"))
                      printf(mfilename);
                      printf(" no significative possible move:(%d,%d)",rotationToDo,lenToDo);
                      printf(ctime(time()));
                endif        
                [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOff);
                 if (action=="inMove" || action=="stop..")
                    printf(mfilename);
                    printf(" stop required after Rotate : recode %d, action (%d,%d)  *** ",retCode,action(1),action(2));
                    printf(ctime(time())); 
                    apRobot = setfield(apRobot,"automatonState",[lost,automatonState(2),automatonState(3)]);   
                    count=maxRetry;
                 endif
                 if (retCode==0)
                   lastMesurmentReliable=true;
                 else
                    lastMesurmentReliable=false;
                 endif                
          case([localizing,notLocalized,moving])  
                  [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,(plotValue>=3));
                  if (retCodeMove!=robot.moveKoDueToNotEnoughSpace && retCodeMove!=robot.moveUnderLimitation && retCodeMove!=robot.moveKoDueToWheelStopped)
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
                   else
                    count--;
                   endif  
                 if (retCodeMove==0)
                   lastMesurmentReliable=true;
                 else
                   lastMesurmentReliable=false;
                 endif  
            
          case([locked,notLocalized,atRest])  
                  printf(mfilename);
                  printf(" locked state:(%d,%d,%d)  *** ",automatonState(1),automatonState(2),automatonState(3));
                  printf(ctime(time())); 
                  randomChoice=true;
                  [apRobot,robot,retCode,action]=ApRobotScan360(apRobot,robot,(plotValue>=1));                    
                  apRobot = setfield(apRobot,"automatonState",[localizing,determining,scanned]);  % force to skip determination step     
   
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
                 printf(mfilename);
                 printf(" end retry:%d  *** ",count);
                 printf(ctime(time()));
                 %traceDet
                 %histDeltaDist
                 %traceLoc
        if (realMode)
          if (EchoLoc(1)==-1)
            robot.Horn(3);  % horn x seconds
            else
              for i=1:3
                  robot.Horn(1);  % horn x seconds
                  pause(2);
              end
          endif       
      endif
      return;
endfunction