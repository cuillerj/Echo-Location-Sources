function [actionList,retCode] = GmapAction4(apRobot,currentLocation,nextLocation,parameters)

    retCode=0;
    actionList=[];
    instruction=parameters(1);
    crossPoint=parameters(2:3);
    dist=parameters(4);
    lineHeading=parameters(5);
    optimalCrossX=parameters(6:7);
    shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter");
    shiftEchoVsRotationCenter=shiftEchoVsRotationCenter/10;
    if (dist>apGet(apRobot,"pathMaxStraightLenght")+20)
        printf(mfilename);
        printf(" far for zone *** ");
        printf(ctime(time()));
        return
    else
        if (instruction==1)          
            if(crossPoint(1)>=optimalCrossX(1) && crossPoint(1)<=optimalCrossX(2))    % straight trajectory cross correctly the line        
                printf(mfilename);
                printf(" must run backward over the obstacle *** ")
                printf(ctime(time()));
                head=(atan2(nextLocation(2)-currentLocation(2),nextLocation(1)-currentLocation(1))+pi())*180/pi;
                rot=-currentLocation(3)+head;
                [apRobot,possible,rotation,step] = GmapCheckTurnRound(apRobot,head);
  %              [rotation,lenToDo,forward] = ApOptimizeMoveToDo(rot,0,false,shiftEchoVsRotationCenter)
                actionList=[1,round(rot),round(-sqrt((nextLocation(2)-currentLocation(2))^2+(nextLocation(1)-currentLocation(1))^2))];
            else
                nextStep=[round(mean(optimalCrossX)),currentLocation(2)];
                printf(mfilename);
                printf(" goto (%d,%d) *** ",nextStep(1),nextStep(2));
                printf(ctime(time()));
                actionList=[2,nextStep];
            endif
        endif
    
    endif
    
    endfunction