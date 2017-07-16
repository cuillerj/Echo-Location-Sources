function [apRobot] = ApCreateLocatedParticles(apRobot,particlesNumber,plotOn)
%{
create particles n=(prob/particlesNumber) located at (posX,posY,heading) with x y h precision
create particles n=(particlesNumber - n) located anywhere inide carto
if posX <0 or posY <0 particles are created anywhere
if heading <0 particles are created in any orientation
if prob > 0 some particles are created according to posX posY and heading 
if posX and posY <0 and heading > 0 prob will not be used
%}
  degreUnit=0;
  radianUnit=1;
  debugOn=1;
  debugOff=0;
  carto=apGet(apRobot,"carto");
  img=apGet(apRobot,"img");
  loc=apGet(apRobot,"location");
  posX=loc(1);
  posY=loc(2);
  heading=loc(3);
  prob=apGet(apRobot,"locationProb");
  pw=0;
  xPrecision=10;
  yPrecision=10;
  hPrecision=10;
  sigmaDist=2;  %2
  if (posX==-1 && prob==100)      % means robot is only nort oriented
    sigmaHeading=2;
  else
    sigmaHeading=0.5;
  endif
  particles=[];
  [x,y]=size(carto);
  lim=particlesNumber*prob/100;
  if (posX<0 || posY<0)
    lim=0
  endif
  i=0;
  while (i<=lim)             % create some particles according to posX posY and heading
    px=normrnd(posX,sigmaDist);
    py=normrnd(posY,sigmaDist);
    po=normrnd(heading,sigmaHeading);
    i=i+1;
    particles=[particles;[px,py,po,pw]];
  end
  i=0;
  if (lim==0 && heading >=0)   
    lim=(particlesNumber-lim)-1;
    while (i<=lim)        % create some particles anywhere in any direction
    px=randi(x);
    py=randi(y);
    po=heading+randi((100+hPrecision-prob))-(100+hPrecision+1-prob)/2;
    if (ApQueryCartoAvailability(apRobot,[px,py,po*pi()/180],radianUnit,debugOff)==1)
      i=i+1;
      particles=[particles;[px,py,po,pw]];
    endif
    end
    lim=0;
  else
    lim=(particlesNumber-lim)-1;
  endif
  while (i<=lim)        % create some particles anywhere in any direction
    px=randi(x);
    py=randi(y);
    po=randi(361)-1;
    if (ApQueryCartoAvailability(apRobot,[px,py,po*pi()/180],radianUnit,debugOff)==1)
      i=i+1;
      particles=[particles;[px,py,po,pw]];
    endif
  end
  apRobot = setfield(apRobot,"particles",particles);
  apRobot = setfield(apRobot,"lastParticles",particles);
%  save ("-mat4-binary","particles.mat","particles")
  if (plotOn)
    figure();
    title ("created particles");
    hold on;
    imshow(img,[])
    [a,b]=size(img);
    axis([1,b,1,a],"on","xy");
    for i=1:particlesNumber
      plot(particles(i,1),particles(i,2))
    end
    hold off;
  endif
  return
endfunction
