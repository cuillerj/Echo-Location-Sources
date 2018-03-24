function [apRobot,robot] = ApNorthAlignParticles(apRobot,robot,NorthHeading,plotOn)
  particles=apGet(apRobot,"particles");      
  apRobot = setfield(apRobot,"lastParticles",particles); 
  currentNorthOrientationReference=apGet(apRobot,"currentNorthOrientationReference");
  [x,y]=size(particles);
  sigmaHeading=3;
  heading=currentNorthOrientationReference+NorthHeading;
  for i=1:x
    particles(i,3)= normrnd(heading,sigmaHeading);
  endfor
    apRobot = setfield(apRobot,"particles",particles);
  %save ("-mat4-binary","particles.mat","particles")
  if (plotOn)
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
  endif
endfunction
