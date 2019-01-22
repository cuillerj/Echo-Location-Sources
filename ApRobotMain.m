 function [apRobot,robot,EchoLoc,traceLoc] = ApRobotMain(flatLogRegMode,realMode,autoLocalization,plotValue,apRobot,robot)

     if (!exist("flatLogRegMode") && !exist("realMode") && !exist("autoLocalization") && !exist("plotValue"))  % flat logistic regression is default mode 
        printf(mfilename);
        printf(" parameters are: (flatLogRegMode,realMode,autoLocalization,plotValue)")
        printf(ctime(time()))
        return
      endif
      close all; % close all figures
      clc;
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
    if (!exist("autoLocalization"))  % autoLocalization is default mode 
        autoLocalization=1;
    endif

   %simulationMode=1;  % to set simulation 1 on 0 off
    simulationMode=!realMode;

   % printf(ctime(time()))
   % particlesNumber=2000;
    plotOn=true;                 % if true graphics will be provided (reduces performance)
    plotOff=false;               % no graphic
    noiseLevel=0.2;                 % coefficent (float) of noise of simualtion mode 0 no noise
    noiseRetCode=0;                 % boolean (0 move 100% completed 1 move can be incompleted)
    scanNoiseLevel=0.7;             % 0.0 0.5
    noiseRetValue=12;               % range of noised retcode in wich a random retcode is choosen 
    [apRobot,robot] =ApInitApRobot(flatLogRegMode,realMode,apRobot,robot);
    apRobot = setfield(apRobot,"simulationMode",simulationMode);
    apRobot = setfield(apRobot,"realMode",realMode);

     
    if (realMode)
        rcBatch=robot.LaunchBatch();        % call java method to start all batchs
      else
        rcBatch=robot.LaunchSimu();        % call java method to start only simulator
        robot.noiseLevel=noiseLevel;         
        robot.scanNoiseLevel=scanNoiseLevel;          	
        robot.noiseRetCode=noiseRetCode;  
        robot.noiseRetValue=noiseRetValue;   
    endif
    if (rcBatch!=0)
        printf(mfilename);
        printf(" java batch launch issue:%d - Need to restart Octave *** ",rcBatch);
        printf(ctime(time()));
        return
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
          robot.ResetRobotStatus();
    endif
    printf(mfilename);
    if (realMode)     
      target=[385,250,0];
      printf(mfilename);
      printf(" real mode destination (%d,%d) *** ",target(1),target(2))
      printf(ctime(time()));
    else
   %   target = ApGetRadomPosition(apRobot);
      [apRobot,robot,target] = ApGetRandomLocation(apRobot,robot);
      printf(mfilename);
      printf(" Simulation mode random destination (%d,%d) *** ",target(1),target(2)) 
      printf(ctime(time())); 
    endif
    apRobot = setfield(apRobot,"destination",target);     
    stopRequest=false;
    robot.ResetRobotStatus();
    loopId=1;
    if (!autoLocalization)
      [apRobot,robot,stopRequested] = ApDetermineCurrentLocation(apRobot,robot,1);
      if(stopRequested)
        return;
      endif
       apRobot = setfield(apRobot,"automatonState",[targeting,localized,atRest]); 
      %return;
    endif
    while(!stopRequest)
     automatonState=apGet(apRobot,"automatonState");
        if (automatonState(1)==lost)
             printf(mfilename);
             printf(" robot lost *** ");
             printf(ctime(time()))
             stopRequest=true;
             return;  
        endif       
        switch(automatonState(2)) % localization status
          case{notLocalized,localisationLost}
              automatonState=apGet(apRobot,"automatonState");
              apRobot = setfield(apRobot,"automatonState",[initial,notLocalized,atRest]); 
              [apRobot,robot,EchoLoc,traceLoc,locRetCode] = ApEchoLocalization(apRobot,robot,flatLogRegMode,realMode,plotValue,loopId);
              apRobot = setfield(apRobot,"location",EchoLoc); 
              if (locRetCode==-1)
                return
              endif
          case(localized)
              printf(mfilename);
              printf(" robot localised *** ");
              printf(ctime(time()))
            %  apRobot = setfield(apRobot,"locationProb",100);     
              [apRobot,robot,EchoLoc,retCode] = ApGotoTarget(apRobot,robot,flatLogRegMode,plotValue)
              if (retCode==-1)
                return
              endif
              automatonState=apGet(apRobot,"automatonState");
              if (automatonState(1)==gotTarget)
                    printf(mfilename);
                    printf(" robot got the target *** ");
                    printf(ctime(time()))       
                return;
              else
                    printf(mfilename);
                    printf(" robot locked or lost during the traject to the target *** ");
                    printf(ctime(time()))   
                    apRobot = setfield(apRobot,"automatonState",[automatonState(1),notLocalized,atRest]);              
              endif
              
           otherwise
               switch(automatonState(1)) %   main status
                      case{initial,localizing,locked}
                              printf(mfilename);
                              printf(" enter new localization phase *** ");
                              printf(ctime(time()))
                              apRobot = setfield(apRobot,"automatonState",[initial,notLocalized,atRest]); 
                              stopRequest=false;
                              loopId++;
                              pause(5);
      #                        break;                              
                      case{targeting}
                              printf(mfilename);
                              printf(" enter going to the target phase *** ");
                              printf(ctime(time()))
                              %stopRequest=true;
                              pause(1) 
   #                           break;                             
                      case{gotTarget}
                              printf(mfilename);
                              printf(" robot got the target *** ");
                              printf(ctime(time()))        
                              return;          
                      case{locked}
                              printf(mfilename);
                              printf(" robot locked *** ");
                              printf(ctime(time())) 
                              stopRequest=true;       
                              return;  
                endswitch
   #             break;
          endswitch
     
     endwhile
     return;
   
   endfunction