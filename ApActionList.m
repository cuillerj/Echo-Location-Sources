function [actionString] = ApActionList(apRobot,robot,action)
    # robot actions list
  actionString=  apGet(apRobot,"actionList")(action+1);
  endfunction