function [apRobot,robot] = ApMoveParticles(apRobot,robot,rotation,distance,plotOn)
  %{
  Repere XY l 'espace
  Repere AB le robot
  C reference position du robot (echo) C_AB(-shiftEchoVsRotationCenter,0,1)
  P centre de rotation du robot (milieu des roues) O_AB=(0,0) P_XY=(loaction(1),location(2),1)
  %}

rotation=mod(rotation+360,360);
particles=apGet(apRobot,"particles");      
apRobot = setfield(apRobot,"lastParticles",particles); 
location=apGet(apRobot,"location");  
img=apGet(apRobot,"img");  
shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter")/10;           
sigmaDist=abs(2*distance/100+3);
sigmaRot=0.9; %1
sigmaHeading=0.5; %0.5
[x,y]=size(particles);
CP_AB=[-shiftEchoVsRotationCenter;0;1];
PC_AB=[shiftEchoVsRotationCenter;0;1];
for i=1:x
	dist=normrnd(distance,sigmaDist); % add noise
	if (rotation==0)
		rot=normrnd(0,sigmaHeading);
	else
		rot=normrnd(rotation,sigmaRot);
	endif
  % apply rotation
  currentH=particles(i,3)*pi/180;
  currentP_XY=[particles(i,1);particles(i,2);1];
  rotGrad=rot*pi/180;
  newH=currentH+rotGrad;
  R0_XY_AB=[cos(currentH),-sin(currentH),0;sin(currentH),cos(currentH),0;0,0,1];
  C0_XY=R0_XY_AB*CP_AB;
  R_XY_C=[cos(rotGrad),-sin(rotGrad),0;sin(rotGrad),cos(rotGrad),0;0,0,1];
  %C1_XY= R_XY_C*C0_XY
  R1_XY_AB=[cos(newH),-sin(newH),0;sin(newH),cos(newH),0;0,0,1];
  T1_XY=[1,0,(R1_XY_AB*PC_AB)(1);0,1,(R1_XY_AB*PC_AB)(2);0,0,1];
  T2_XY=[1,0,currentP_XY(1);0,1,currentP_XY(2);0,0,1];
  P_XY=T2_XY*T1_XY*C0_XY;
  % apply translation
  T3_XY=[1,0,dist*cos(newH);0,1,dist*sin(newH);0,0,1];
  P_XY=T3_XY*P_XY;
  particles(i,1)=P_XY(1);
  particles(i,2)=P_XY(2);
	particles(i,3)=mod(newH*180/pi(),360);
endfor
  apRobot = setfield(apRobot,"particles",particles);
%save ("-mat4-binary","particles.mat","particles")
  plotRatio=apGet(apRobot,"plotRatio");
if (plotOn)
  %figureNumber=get (0, "currentfigure");
  [apRobot,figureNumber] = ApPlotParticles(apRobot,plotRatio,"moved particles >");
   printf(mfilename);
   printf(" figure :%d ***  ",figureNumber);
   printf(ctime(time()));	

endif
endfunction
