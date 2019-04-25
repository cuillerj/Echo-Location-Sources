function [apRobot] = ApDefineAutomatonActionList(apRobot)
      %automatonActionList={"moveStraight";"rotate";"northAlign";"scan360";"determine";"pingFB";"checkTarget";"checkLocation";"acrossPath"};
      automatonActionList = ApAutomatonActionList(apRobot);
      apRobot = setfield(apRobot,"automatonActionList", automatonActionList); % automaton actions list
      apRobot = setfield(apRobot,"automatonMoveStraight",find(strcmp(automatonActionList,"moveStraight")));
      apRobot = setfield(apRobot,"automatonRotate",find(strcmp(automatonActionList,"rotate")));
      apRobot = setfield(apRobot,"automatonNorthAlign",find(strcmp(automatonActionList,"northAlign")));
      apRobot = setfield(apRobot,"automatonScan360",find(strcmp(automatonActionList,"scan360")));
      apRobot = setfield(apRobot,"automatonDetermine",find(strcmp(automatonActionList,"determine")));
      apRobot = setfield(apRobot,"automatonPingFB",find(strcmp(automatonActionList,"pingFB")));
      apRobot = setfield(apRobot,"automatonCheckTarget",find(strcmp(automatonActionList,"checkTarget")));
      apRobot = setfield(apRobot,"automatonCheckLocation",find(strcmp(automatonActionList,"checkLocation")));
      apRobot = setfield(apRobot,"automatonAcrossPath",find(strcmp(automatonActionList,"acrossPath")));
 endfunction