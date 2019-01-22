function [apRobot,robot,easy] = TestGoToGoal2Wheels(apRobot,robot);
  %{
  
  simulation d'un go to goal robot 
  les contraintes de rotation sont declinees a partir de maxWheelRPS et minWheelRPS
  
  start manual simulation mode before
  
  example
   apRobot = setfield(apRobot,"destination",[-1,1,0]);
   apRobot = setfield(apRobot,"location",[2,2,270]);
  
  %}
 % apRobot = setfield(apRobot,"destination",[50,50,0]);

  maxWheelRPS=apGet(apRobot,"maxWheelRPS")/100
  minWheelRPS=apGet(apRobot,"minWheelRPS")/100
  avgWheelRPS=(maxWheelRPS+minWheelRPS)/2
  leftRPS=avgWheelRPS;
  rightRPS=avgWheelRPS;
  iRobotWidth=apGet(apRobot,"iRobotWidth");
  robotRadius=iRobotWidth/2;
  robotLenght=apGet(apRobot,"frontLenght")+ apGet(apRobot,"backLenght");
  wheelDiameter=(apGet(apRobot,"iLeftWheelDiameter")+apGet(apRobot,"iLeftWheelDiameter"))/200;
  RB=(robotRadius)*(minWheelRPS+maxWheelRPS)/(maxWheelRPS-minWheelRPS)
  robotSpeed=avgWheelRPS*pi()*wheelDiameter;
  v=robotSpeed
  DtR=(RB*2*pi())/robotSpeed;
  deltaT=0.1;
  oMax=(2*pi()/DtR)*deltaT;
  gotTarget=false;
  reachable=true;
  goal=apGet(apRobot,"destination")
  loc=apGet(apRobot,"location");
  reachLimit=5;
  figure()
  figNumber=get (0, "currentfigure");
  axis("square");
  axis equal;
  hold on
  plot(goal(1),goal(2),'marker','*','markersize',15)
  h=loc(3)*pi()/180;
  center1=[loc(1)+RB*cos(h+pi()/2),loc(2)+RB*sin(h+pi()/2)];
  center2=[loc(1)+RB*cos(h-pi()/2),loc(2)+RB*sin(h-pi()/2)];
  centerVector=[center1(1)-center2(1),center1(2)-center2(2)];
  goalVector=[goal(1)-loc(1),goal(2)-loc(2)];
  prod=centerVector*goalVector';
%  timeToReachMinSpeed=1.8;
  
  distGoalC1=sqrt((center1(1)-goal(1))^2+(center1(2)-goal(2))^2);
  distGoalC2=sqrt((center2(1)-goal(1))^2+(center2(2)-goal(2))^2);
  precision=e^-10;
  if(prod>0+precision)
    plot(center1(1),center1(2),'marker','o','markersize',10);
  elseif (prod<0-precision)
    plot(center2(1),center2(2),'marker','o','markersize',10);
  else
    plot(center2(1),center2(2),'marker','o','markersize',10);
    plot(center1(1),center1(2),'marker','o','markersize',10);   
  endif
  if (distGoalC1<RB-reachLimit || distGoalC2<RB-reachLimit)
    printf("not easy \n")
    easy=false;
  else 
    printf(" easy \n")
    easy=true;
  endif
  if (h>pi())
    h=h-2*pi();
  endif
  newDist=inf;
  t0=time;
  [apRobot,robot,figureNumber] = ApDrawRobot(apRobot,robot,0);
  idx=0;
  while (!gotTarget && reachable)
    %atan((goal(2)-loc(2))/(goal(1)-loc(1)))
    phi=atan2((goal(2)-loc(2)),(goal(1)-loc(1)));
    pause(deltaT);
    vx=v*cos(h);
    vy=v*sin(h);
    loc(1)=loc(1)+vx*deltaT;
    loc(2)=loc(2)+vy*deltaT;
    error=phi-h;
    if (error>pi())
      error=error-2*pi();
    endif
    delta = ToolAngleDiffGrad(phi,h);
    if (abs(delta)>oMax)
      error=oMax*sign(error);
    endif
 %   if(time-t0>timeToReachMinSpeed)
 %{
   RB=(robotRadius)*(minWheelRPS+maxWheelRPS)/(maxWheelRPS-minWheelRPS)
  DtR=(RB*2*pi())/robotSpeed;
  RB*2*pi()=DtR*robotSpeed
  RB=DtR*robotSpeed/(2*pi())
  RB/robotRadius=(lowRPS+highRPS)/(highRPS-lowRPS) = 2*avgRPS/(avgRPS+deltaRPS-(avgRPS-deltaRPS)) = avgRPS/deltaRPS
  deltaRPS=avgRPS*robotRadius/RB
   oMax=(2*pi()/DtR)*deltaT;   oMax/deltaT= (2*pi()/DtR)   DtR=(2*pi()*deltaT)/oMax
  %}
   % DtR=2*pi()*deltaT/error
    if (error!=0)
      expectedRB=(deltaT*robotSpeed)/error;
    else
    expectedRB=0;
    endif
    if (expectedRB!=0)
      deltaRPS=avgWheelRPS*robotRadius/expectedRB;
    else
      deltaRPS=0;
    endif
    omegaR=avgWheelRPS+deltaRPS;
    omegaL=avgWheelRPS-deltaRPS;
    h=h+error;
  %  endif
    if (h>pi())
      h=h-2*pi();
    endif
    figure(figNumber);
    plot(loc(1),loc(2),'marker','+','markersize',15)
    if(mod(idx,5)==0)
      apRobot = setfield(apRobot,"location",[loc(1),loc(2),h*180/pi()]);
      [apRobot,robot,figureNumber] = ApDrawRobot(apRobot,robot,0,figureNumber);
    endif
    idx++;
    prevDist=newDist;
    newDist=sqrt((goal(2)-loc(2))^2+(goal(1)-loc(1))^2);
    if (newDist<=reachLimit)
      gotTarget=true
      printf("duration:%d\n",time-t0)
    endif
    if (newDist>prevDist && abs(error)==oMax)
      %reachable=false
    endif
  endwhile
    printf("heading:%d degrés \n",h*180/pi())
  hold off

  endfunction