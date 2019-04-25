function [apRobot,robot,actionList,retCode] = ApCheckSpecificInstruction(apRobot,robot,nextLocation,plotOn)
    %{
        actionList(1) = 1 > rotate actionList(2) and move actionList(3)
        actionList(1) = 2 > goto ( actionList(2) , actionList(3))
    %}
    if (!exist("plotValue"))
      plotValue=1;
    endif
    retCode=0;
    currentL=apGet(apRobot,"location");
    rotationType=2;
    actionList=[];
  %    carto=apGet(apRobot,"carto");
   %   shitfCartoX=apGet(apRobot,"shitfCartoX");
   %   shitfCartoY=apGet(apRobot,"shitfCartoY");
    %  carto(nextLocation(1)+shitfCartoX,nextLocation(2)+shitfCartoY)
      zones=apGet(apRobot,"zonesList");
      [a,b]=size(zones);
      for (i=1:b)
             x=zones(i);
             fct=x(1).fct;
             act=x(1).act;
             [instructionNumber,parameters,retCode] =feval(fct,currentL,nextLocation);
            if (instructionNumber!=0)
                printf(mfilename);
                printf("  some specific instruction for zone:%d *** ",i);
                printf(ctime(time()));
                break;
              endif
      endfor
      if (instructionNumber!=0)
        parameters=[instructionNumber,parameters];
        printf(mfilename);
        printf(" action:%d*** ",instructionNumber);
        printf(ctime(time()));
        [actionList,retCode] = feval (act,apRobot,currentL,nextLocation,parameters);
      endif
       
    endfunction