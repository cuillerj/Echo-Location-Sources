function [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,action,debugOn)
  if (!exist("debugOn"))
    debugOn=false;
  endif
  retCode=-3;
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
  aStatus=["atRest";"NOrient";"moving";"scanned";"scanned"];
  %{
  actions list
  %}
  moveStraight=1;
  rotate=2;
  northAlign=3;
  scan360=4;
  determine=5;
  pingFB=6;
  actionList=["moveStraight";"rotate";"northAlign";"scan360";"determine";"pingFB"];
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
  if (action(2)==speedInconstistency)
    action(2)=wheelStopped;
  endif
  alphabet=[[moveStraight,normal];[moveStraight,timeout];[moveStraight,notEnoughSpace];[moveStraight,wheelStopped];[moveStraight,obstacle];[moveStraight,underLimitation];[moveStraight,pending] %1-7
            [rotate,normal];[rotate,timeout];[rotate,notEnoughSpace];[rotate,wheelStopped];[rotate,obstacle];[rotate,underLimitation];[rotate,pending]; %8-14
            [northAlign,normal];[northAlign,timeout];[northAlign,notEnoughSpace];[northAlign,wheelStopped];[northAlign,obstacle];[northAlign,underLimitation];[northAlign,pending]; %15-21
            [scan360,normal];[scan360,timeout];[scan360,pending];  %22-24
            [pingFB,normal];[pingFB,timeout];  %25-26
            [determine,determined];[determine,!determined]   %27-28
            ];
  statesListL=[[initial,notLocalized,atRest];  %1
            [localizing,notLocalized,atRest];[localizing,notLocalized,NOrient];[localizing,notLocalized,moving];[localizing,notLocalized,scanned]; %2-5
            [locked,notLocalized,atRest];[locked,notLocalized,scanned];[locked,notLocalized,moving]; %6-8
            [targeting,localized,atRest];[lost,notLocalized,atRest]; %9-10
            [localizing,determining,scanned];  %11
            ];  

            

  transitionsListL=[[4,1,2];[4,2,10];[4,3,6];[4,4,4];[4,5,2];[4,6,2];[4,7,4];  %  1-7 move straight
                    [11,8,4];[11,9,10];[11,10,6];[11,11,4];[11,12,4];[11,13,4];[11,14,5];  % 8-14 rotate
                    [1,15,3];[1,16,10];[1,17,6];[1,18,3];[1,19,3];[1,20,3];[1,21,2];  % 15-21 -northAlign                    
                    [2,15,3];[2,16,10];[2,17,6];[2,18,3];[2,19,3];[2,20,3];[2,21,2];  % 22-28 -northAlign
                    [3,22,5];[3,23,6];[3,24,3];           % 29-31 scan360
                    [5,27,9];[5,28,11];          % 32-33 determine
                    [8,4,11];
                    [11,27,9];[11,28,11]
                   ];   %(stateIdx,alphabetIdx,newStateIdx)
    
  statesListT=[[initial,localized,atRest]]; 
  statesListK=[[locked,notLocalized,atRest];[locked,notLocalized,scanned];[locked,notLocalized,moving];[localizing,notLocalized,atRest];[lost,notLocalized,atRest]]; 
  transitionsListK=[[5,3,2];[5,6,2];
                    [5,10,2];[5,13,2];
                    [5,17,10];[5,20,10]];
  
  
  automatonState=apGet(apRobot,"automatonState");  
  newState=[mStatus(automatonState(1),:);lStatus(automatonState(2),:);aStatus(automatonState(3),:)];
  if ((automatonState(1)==localizing) || (automatonState(1)==initial && automatonState(2)==notLocalized))
      statesList=statesListL;
      statesListNumber=size(statesList,1);
      transitionsList=transitionsListL;
  endif
  if ((automatonState(1)==locked))
      statesList=statesListK;
      statesListNumber=size(statesList,1);
      transitionsList=transitionsListK;
  endif
  statesListNumber=size(statesList,1);
  alphabetNumber=size(alphabet,1);
  transitionsNumber=size(transitionsList,1);
  alphabetIdx=0; 
  if (debugOn)
     printf(mfilename);
     printf(" previous automaton status: ( %s %s %s) action (%s,%d) *** ",(newState(1,:)),(newState(2,:)),(newState(3,:)),actionList(action(1),:),action(2))
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
        printf(" new automaton status: ( %s %s %s ) *** ",newState(1,:),newState(2,:),newState(3,:))
        printf(ctime(time()));	
        break;
      endif
    end
  apRobot = setfield(apRobot,"automatonRC",retCode);    
  endfunction