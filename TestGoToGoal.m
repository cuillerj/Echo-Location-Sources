function [apRobot,robot] = TestGoToGoal(apRobot,robot);
  %{
  
  simulation d'un go to goal robot
  les contraintes de rotation sont ecprimmees par maxWheelRPS et minWheelRPS
  
  start manual simulation mode before
  
  example
   apRobot = setfield(apRobot,"destination",[-1,1,0]);
   apRobot = setfield(apRobot,"location",[2,2,270]);
  
  %}
  apRobot = setfield(apRobot,"destination",[184,184,0]);


  maxWheelRPS=apGet(apRobot,"maxWheelRPS")/100
  minWheelRPS=apGet(apRobot,"minWheelRPS")/100
  robotRadius=apGet(apRobot,"iRobotWidth")/2;
  wheelDiameter=(apGet(apRobot,"iLeftWheelDiameter")+apGet(apRobot,"iLeftWheelDiameter"))/200;
  RB=(robotRadius)*(minWheelRPS+maxWheelRPS)/(maxWheelRPS-minWheelRPS)
  robotSpeed=(minWheelRPS*pi()+maxWheelRPS*pi())*wheelDiameter/2
  v=robotSpeed;
  DtR=(RB*2*pi())/robotSpeed;
  deltaT=0.1;
  oMax=(2*pi()/DtR)*deltaT;
  gotTarget=false;
  reachable=true;
  goal=apGet(apRobot,"destination")
  loc=apGet(apRobot,"location");
  reachLimit=2;
  figure()
  axis("square");
  hold on
  plot(goal(1),goal(2),'marker','*','markersize',15)
  h=loc(3)*pi()/180;
  center1=[loc(1)+RB*cos(h+pi()/2),loc(2)+RB*sin(h+pi()/2)];
  center2=[loc(1)+RB*cos(h-pi()/2),loc(2)+RB*sin(h-pi()/2)];
  centerVector=[center1(1)-center2(1),center1(2)-center2(2)];
  goalVector=[goal(1)-loc(1),goal(2)-loc(2)];
 % headingVector=[cos(h),sin(h)]
%  proj1=(headingVector*goalVector')*goalVector
  prod=centerVector*goalVector';

  
  distGoalC1=sqrt((center1(1)-goal(1))^2+(center1(2)-goal(2))^2);
  distGoalC2=sqrt((center2(1)-goal(1))^2+(center2(2)-goal(2))^2);
  if(prod>0)
    plot(center1(1),center1(2),'marker','o','markersize',10);
  elseif (prod<0)
    plot(center2(1),center2(2),'marker','o','markersize',10);
  else
    plot(center2(1),center2(2),'marker','o','markersize',10)
    plot(center1(1),center1(2),'marker','o','markersize',10);   
  endif
  if (distGoalC1<RB-reachLimit || distGoalC2<RB-reachLimit)
    printf("not easy \n")
  else 
    printf(" easy \n")
  endif
  if (h>pi())
    h=h-2*pi();
  endif
  newDist=inf;
  t0=time;
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
    h=h+error;
    if (h>pi())
      h=h-2*pi();
    endif
    plot(loc(1),loc(2),'marker','+','markersize',15)
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