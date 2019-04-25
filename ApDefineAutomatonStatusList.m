function [apRobot] = ApDefineAutomatonStatusList(apRobot)
      %automatonActionList={"moveStraight";"rotate";"northAlign";"scan360";"determine";"pingFB";"checkTarget";"checkLocation";"acrossPath"};
      [mStatus,lStatus,aStatus] = ApAutomatonStatusList(apRobot);
      
      apRobot = setfield(apRobot,"automatonMainStatus", mStatus); % automaton status list
      apRobot = setfield(apRobot,"automatonLocalizationStatus", lStatus); % automaton status list
      apRobot = setfield(apRobot,"automatonActionStatus", aStatus); % automaton status list
      
      apRobot = setfield(apRobot,"automatonInitial",find(strcmp(mStatus,"initial")));
      apRobot = setfield(apRobot,"automatonLocalizing",find(strcmp(mStatus,"localizing")));
      apRobot = setfield(apRobot,"automatonTargeting",find(strcmp(mStatus,"targeting")));
      apRobot = setfield(apRobot,"automatonGotTarget",find(strcmp(mStatus,"gotTarget")));
      apRobot = setfield(apRobot,"automatonLocked",find(strcmp(mStatus,"locked")));
      apRobot = setfield(apRobot,"automatonLost",find(strcmp(mStatus,"lost")));
      
      apRobot = setfield(apRobot,"automatonNotLocalized",find(strcmp(lStatus,"notLocalized")));
      apRobot = setfield(apRobot,"automatonLocalized",find(strcmp(lStatus,"localized")));
      apRobot = setfield(apRobot,"automatonLocalisationLost",find(strcmp(lStatus,"localisationLost")));
      apRobot = setfield(apRobot,"automatonDetermining",find(strcmp(lStatus,"determining")));
      
      apRobot = setfield(apRobot,"automatonAtRest",find(strcmp(aStatus,"atRest")));      
      apRobot = setfield(apRobot,"automatonNOrient",find(strcmp(aStatus,"NOrient")));    
      apRobot = setfield(apRobot,"automatonMoving",find(strcmp(aStatus,"moving")));    
      apRobot = setfield(apRobot,"automatonScanned",find(strcmp(aStatus,"scanned")));    
      
 endfunction