function [apRobot,robot] = ApMoveParticles(apRobot,robot,rotation,distance,plotOn)
rotation=mod(rotation+360,360);
particles=apGet(apRobot,"particles");      
apRobot = setfield(apRobot,"lastParticles",particles); 
location=apGet(apRobot,"location");  
img=apGet(apRobot,"img");  
shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter")/10;           
sigmaDist=abs(2*distance/100+3);
sigmaRot=1.1; %1
sigmaHeading=0.75; %0.5
[x,y]=size(particles);
for i=1:x
	dist=normrnd(distance,sigmaDist); % add noise
	if (rotation==0)
		rot=normrnd(0,sigmaHeading);
	else
		rot=normrnd(rotation,sigmaRot);
	endif
  currentH=particles(i,3)*pi/180;
	newH=currentH+rot*pi/180;
	particles(i,1)=particles(i,1)+(dist+shiftEchoVsRotationCenter)*cos(newH)-shiftEchoVsRotationCenter*cos(currentH);  
	particles(i,2)=particles(i,2)+(dist+shiftEchoVsRotationCenter)*sin(newH)-shiftEchoVsRotationCenter*sin(currentH);
	particles(i,3)=mod(newH*180/pi,360);
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
  %{
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
	figure();
	title ("moved particles");
	hold on;
	imshow(img,[])
	[a,b]=size(img);
	axis([1,b,1,a],"on","xy");
	for i=1:x
		plot(particles(i,1)+shitfCartoX,particles(i,2)+shitfCartoY)
	end
	hold off;
  %}
endif
endfunction
