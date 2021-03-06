function [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,action,debugOn)
  %{
  x diffent stateList are used corresponding to different major phases (localizing, targeting..)
  robot state is [mainStatus,LocalizationStatus,movingStatus]
  alphabet is [action, retCode] 
  transition (robotState, alphabet) => new robotState
  %}

  if (!exist("debugOn"))
    debugOn=false;
  endif
  retCode=-3;
  %{
  main status
  %}
  [mStatus,lStatus,aStatus] = ApAutomatonStatusList(apRobot);
  initial=apGet(apRobot,"automatonInitial");  
  localizing=apGet(apRobot,"automatonLocalizing"); 
  targeting=apGet(apRobot,"automatonTargeting");
  gotTarget=apGet(apRobot,"automatonGotTarget");
  locked=apGet(apRobot,"automatonLocked");
  lost=apGet(apRobot,"automatonLost");

  %mStatus={"initial";"localizing";"targeting";"gotTarget";"locked";"lost"};
   %{
  localization status
  %} 
  notLocalized=apGet(apRobot,"automatonNotLocalized");
  localized=apGet(apRobot,"automatonLocalized");
  localisationLost=apGet(apRobot,"automatonLocalisationLost");
  determining=apGet(apRobot,"automatonDetermining");
  %lStatus={"notLocalized";"localized";"localisationLost";"determining"};
  %{
  action status
  %}

  atRest=apGet(apRobot,"automatonAtRest");
  NOrient=apGet(apRobot,"automatonNOrient");
  moving=apGet(apRobot,"automatonMoving");
  scanned=apGet(apRobot,"automatonScanned");
  %aStatus={"atRest";"NOrient";"moving";"scanned";"scanned"};
  %{
  actions list
  %}
  moveStraight=apGet(apRobot,"automatonMoveStraight");
  rotate=apGet(apRobot,"automatonRotate");
  northAlign=apGet(apRobot,"automatonNorthAlign");
  scan360=apGet(apRobot,"automatonScan360");
  determine=apGet(apRobot,"automatonDetermine");
  pingFB=apGet(apRobot,"automatonPingFB");
  checkTarget=apGet(apRobot,"automatonCheckTarget");
  checkLocation=apGet(apRobot,"automatonCheckLocation");
  acrossPath=apGet(apRobot,"automatonAcrossPath");
  actionList = apGet(apRobot,"automatonActionList");
  %actionList={"moveStraight";"rotate";"northAlign";"scan360";"determine";"pingFB";"checkTarget";"checkLocation";"acrossPath"};
    %{
  return code list
  %}
  normal=0;
  timeout=-1;
  pending=99;
  determined=1;
  notEnoughSpace=robot.moveKoDueToNotEnoughSpace;
  wheelStopped=robot.moveKoDueToWheelStopped;
  obstacle=robot.moveKoDueToObstacle;
  underLimitation=robot.moveUnderLimitation;
  speedInconstistency=robot.moveWheelSpeedInconsistancy;
  PathKoDueToWheelStopped=robot.moveAcrossPathKoDueToWheelStopped;
  PathKoDueToObstacle=robot.moveAcrossPathKoDueToObstacle;
  PathKoDueToNotEnoughSpace=robot.moveAcrossPathKoDueToNotEnoughSpace;
  PathKoDueToNotFindingStart =robot.moveAcrossPathKoDueToNotFindingStart;
  PathKoDueToNotFindingEntry=robot.moveAcrossPathKoDueToNotFindingEntry;
  PathKoDueToNotFindingExit=robot.moveAcrossPathKoDueToNotFindingExit;
 
  
%  if (action(1)==moveStraight && action(2)==speedInconstistency)
 %   action(2)=wheelStopped;
 % endif

  alphabet=[[moveStraight,normal];[moveStraight,timeout];[moveStraight,notEnoughSpace];[moveStraight,wheelStopped];[moveStraight,obstacle];[moveStraight,underLimitation];[moveStraight,pending];[moveStraight,speedInconstistency] %1-8
            [rotate,normal];[rotate,timeout];[rotate,notEnoughSpace];[rotate,wheelStopped];[rotate,obstacle];[rotate,underLimitation];[rotate,pending]; %9-15
            [northAlign,normal];[northAlign,timeout];[northAlign,notEnoughSpace];[northAlign,wheelStopped];[northAlign,obstacle];[northAlign,underLimitation];[northAlign,pending]; %16-22
            [scan360,normal];[scan360,timeout];[scan360,pending];  %23-25
            [pingFB,normal];[pingFB,timeout];  %26-27
            [determine,determined];[determine,0];   %28-29
			      [checkTarget,false];[checkTarget,true];  %30-31
			      [checkLocation,true];[checkLocation,false]; %32-33
            [acrossPath,normal];[acrossPath,PathKoDueToWheelStopped] ;[acrossPath,PathKoDueToObstacle];[acrossPath,PathKoDueToNotEnoughSpace];[acrossPath,PathKoDueToNotFindingStart];[acrossPath,PathKoDueToNotFindingEntry] ;[acrossPath,PathKoDueToNotFindingExit] %34-40
            ];
            
% states list and transitions for localisation phase
  statesListL=[[initial,notLocalized,atRest];  %1
            [localizing,notLocalized,atRest];[localizing,notLocalized,NOrient];[localizing,notLocalized,moving];[localizing,notLocalized,scanned]; %2-5
            [locked,notLocalized,atRest];[locked,notLocalized,scanned];[locked,notLocalized,moving]; %6-8
            [targeting,localized,atRest];[lost,notLocalized,atRest]; %9-10
            [localizing,determining,scanned]  %11
            ];  
            
  transitionsListL=[[4,1,2];[4,2,10];[4,3,6];[4,4,8];[4,5,2];[4,6,2];[4,7,4];[4,8,8];  %  1-8 move straight
                    [11,9,4];[11,10,10];[11,11,6];[11,12,6];[11,13,4];[11,14,4];[11,15,5];  % 9-15 rotate
                    [1,16,3];[1,17,10];[1,18,6];[1,19,11];[1,20,11];[1,21,3];[1,22,2];  % 16-22 -northAlign                    
                    [2,16,3];[2,17,10];[2,18,6];[2,19,6];[2,20,6];[2,21,3];[2,22,2];  % 23-29 -northAlign
                    [3,23,5];[3,24,6];[3,25,3];[6,23,11];[6,24,10];[6,25,11];           % 30-32 scan360
                    [5,28,9];[5,29,11];          % 33-34 determine
                    [8,4,1];
                    [11,23,5];[11,24,10];[11,28,9];[11,29,11];
                    [6,1,10];[6,2,6];[6,3,6];[6,4,6];[6,5,6];[6,6,6];                % locked move straight
                    [10,16,2]
                   ];   %(stateIdx,alphabetIdx,newStateIdx)

 % states list and transitions for targeting phase                  
  statesListT=[[targeting,localized,atRest];[targeting,localized,moving];[targeting,localized,scanned]; % 1-3
				[targeting,notLocalized,scanned];[lost,notLocalized,atRest]; 							% 4-5
				[targeting,determining,scanned];  														% 6
				[gotTarget,localized,atRest];                                 % 7
				[locked,localized,atRest];[locked,notLocalized,atRest];        % 8-9
				[initial,localisationLost,atRest]  %10
			];
  transitionsListT=[[1,1,1];[1,2,5];[1,3,8];[1,4,8];[1,5,8];[1,6,1];[1,7,2];[1,7,8];[1,8,8];  %  1-8 move straight
					[1,9,2];[1,10,5];[1,11,8];[1,12,8];[1,13,8];[1,14,1];[1,15,8]; 			% 9-15 rotate
					[2,1,1];[2,2,5];[2,3,8];[2,4,8];[2,5,8];[2,6,8];[2,7,1];[2,7,2];[2,8,8];  %  16-23 move straight
					[2,9,2];[2,10,5];[2,11,8];[2,12,8];[2,13,8];[2,14,1];[2,15,8]; 			% 24-31 rotate
					[2,26,3];[2,26,2];														% 32-33  ping
					[1,28,1];[1,29,1];														% 34-35 determine
					[2,30,2];[2,31,7];														% 36-37 check target
					[1,23,3];[1,24,5];
					[3,32,1];[3,33,1];
          [1,34,1];[1,35,8];[1,36,8];[1,37,8];[1,38,1];[1,39,1];[1,40,1]
					];    

  % states list and transitions for locked phase
  statesListK=[[locked,notLocalized,atRest];[locked,localized,atRest]; %1-2
               [localizing,notLocalized,atRest];[targeting,localized,atRest] %3-4
               [lost,notLocalized,atRest]  %5
               [locked,notLocalized,moving];[locked,localized,moving]];  %6-7
               
  transitionsListK=[[1,1,3];[2,1,4]; % move ok 
                    [1,2,5];[2,2,5];[1,3,5];[2,3,5];[1,4,5];[2,4,5];[1,5,5];[2,5,5];[1,6,5];[2,6,5];[1,7,1];[2,7,1];[1,8,5];[2,8,5];
                    [6,2,5];[7,2,5];[6,3,5];[7,3,5];[6,4,5];[7,4,5];[6,5,5];[7,5,5];[6,6,5];[7,6,5];[6,7,1];[7,7,1];[6,8,5];[7,8,5];
                    ];
  
  automatonState=apGet(apRobot,"automatonState");
  newState=[mStatus(automatonState(1));lStatus(automatonState(2));aStatus(automatonState(3))];
  if ((automatonState(1)==localizing) || (automatonState(1)==initial && automatonState(2)==notLocalized))
      statesList=statesListL;
      statesListNumber=size(statesList,1);
      transitionsList=transitionsListL;
  endif
  if ((automatonState(1)==locked || automatonState(1)==lost))
      statesList=statesListK;
      statesListNumber=size(statesList,1);
      transitionsList=transitionsListK;
  endif
  if ((automatonState(1)==targeting))
      statesList=statesListT;
      statesListNumber=size(statesList,1);
      transitionsList=transitionsListT;
  endif
  if (!exist("statesList"))
      printf(mfilename);
      printf(" automaton state error:(%d,%d,%d)  *** ",automatonState(1),automatonState(2),automatonState(3));
      printf(ctime(time()));
      retCode=-3;
      return;
  endif
  statesListNumber=size(statesList,1);
  alphabetNumber=size(alphabet,1);
  transitionsNumber=size(transitionsList,1);
  alphabetIdx=0; 
  if (debugOn)
     printf(mfilename);
     printf(" entry automaton status: ( %s %s %s) action (%s,%d) *** ",char(newState(1)),char(newState(2)),char(newState(3)),char(actionList(action(1))),action(2))
     printf(ctime(time()));	
  endif
  letter=0;
  for (i=1:alphabetNumber)
    if (alphabet(i,:)==action)
      letter=i;
      retCode=-2;
      break;
    endif
  end
  state=0; 
  for (i=1:statesListNumber)
    if (statesList(i,:)==automatonState)
      state=i;
      retCode=-1;
      break;
    endif
  end
  
  transition=[state,letter];
    for (i=1:transitionsNumber)
      if (transitionsList(i,1:2)==transition)
        retCode=0;
        newS=statesList(transitionsList(i,3),:);
        apRobot = setfield(apRobot,"automatonState",newS);        
        newState=[mStatus(newS(1),:);lStatus(newS(2),:);aStatus(newS(3),:)];
        printf(mfilename);
        printf(" new automaton status: ( %s %s %s ) *** ",char(newState(1)),char(newState(2)),char(newState(3)))
        printf(ctime(time()));	
        break;
      endif
    end
 if (retCode!=0)
        printf(mfilename);
        printf(" automaton retCode:%d  *** ",retCode)
        printf(ctime(time()));
 endif		
  apRobot = setfield(apRobot,"automatonRC",retCode);    
  endfunction