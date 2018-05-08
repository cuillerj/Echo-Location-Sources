 function [apRobot,robot] = ApRobotMain(flatLogRegMode,realMode,autoLocalization,plotValue)
   
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
   maxRetry=50;
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
    printf(mfilename);
    printf(" real mode :%d *** ",realMode)
    printf(ctime(time()))
   % particlesNumber=2000;
    plotOn=true;                 % if true graphics will be provided (reduces performance)
    plotOff=false;               % no graphic
    noiseLevel=0.3;                 % coefficent (float) of noise of simualtion mode 0 no noise
    noiseRetCode=1;                 % boolean (0 move 100% completed 1 move can be incompleted)
    noiseRetValue=12;               % range of noised retcode in wich a random retcode is choosen
    [apRobot,robot] =ApInitApRobot(flatLogRegMode,realMode);
    apRobot = setfield(apRobot,"simulationMode",simulationMode);
    apRobot = setfield(apRobot,"realMode",realMode);
     
     
     
       if (realMode)
        robot.LaunchBatch();        % call java method to start all batchs
      else
        robot.LaunchSimu();        % call java method to start only simulator
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
    stopRequest=false;
    while(!stopRequest)
     automatonState=apGet(apRobot,"automatonState");
        if (automatonState(1)==lost)
             printf(mfilename);
             printf(" robot lost timeout *** ");
             printf(ctime(time()))
             stopRequest=true;
            break;  
        endif       
        switch(automatonState(2))
          case{1,3}
            automatonState=apGet(apRobot,"automatonState");
            apRobot = setfield(apRobot,"automatonState",[1,automatonState(2),automatonState(3)]); 
            [apRobot,robot,EchoLoc,automatonStateList] = ApEchoLocalization(apRobot,robot,flatLogRegMode,realMode,plotValue)
      
             otherwise
                    switch(automatonState(1))
                      case{initial,localizing,locked}
                              printf(mfilename);
                              printf(" enter localization phase *** ");
                              printf(ctime(time()))
                              stopRequest=true;
                              break;                              
                      case{targeting}
                              printf(mfilename);
                              printf(" enter going to the target phase *** ");
                              printf(ctime(time()))
                              stopRequest=true; 
                              break;                             
                      case{gotTarget}
                              printf(mfilename);
                              printf(" robot got the target *** ");
                              printf(ctime(time()))        
                              stopRequest=true;          
                              break;  
                    endswitch
          endswitch
     
     endwhile
     return;
   
   endfunction