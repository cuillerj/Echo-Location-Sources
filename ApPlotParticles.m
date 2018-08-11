    function [apRobot] = ApPlotParticles(apRobot)
    figure();
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
    img=apGet(apRobot,"img");
    particles=apGet(apRobot,"particles");
    [l,c]=size(particles)
    title (" particles");
    hold on;
    imshow(img,[])
    [a,b]=size(img);
    axis([1,b,1,a],"on","xy");
    for i=1:l
      plot(particles(i,1)+shitfCartoX,particles(i,2)+shitfCartoY)
    end
    hold off;