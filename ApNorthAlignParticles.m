function [apRobot,robot] = ApNorthAlignParticles(apRobot,robot,NorthHeading,plotOn)
  printf(mfilename);
  printf(" align particles: %d  *** ",NorthHeading)
  printf(ctime(time()));		
  particles=apGet(apRobot,"particles");      
  apRobot = setfield(apRobot,"lastParticles",particles); 
  %currentNorthOrientationReference=apGet(apRobot,"currentNorthOrientationReference");
  img=apGet(apRobot,"img"); 
  [x,y]=size(particles);
  sigmaHeading=3;
  heading=mod(NorthHeading,360);
  for i=1:x
    particles(i,3)= normrnd(heading,sigmaHeading);
  endfor
    apRobot = setfield(apRobot,"particles",particles);
  %save ("-mat4-binary","particles.mat","particles")
  if (plotOn)
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
    figure();
    title ("north aligned particles");
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
