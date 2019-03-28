    function [apRobot,figureNumber] = ApPlotParticles(apRobot,plotRatio,head1,figureNumber)
      plotRatio;
    if (!exist("plotRatio"))
         plotRatio=apGet(apRobot,"plotRatio");
    endif
    printf(mfilename);
    printf(" Plot particles ratio:1/%d *** ",plotRatio)
    printf(ctime(time()));
    if (!exist("figureNumber")) 
      keepFigure=false;
      figure();
    else
      keepFigure=true;
      figure(figureNumber)
      hold on;
    endif
    grid minor on;
    figureNumber=get (0, "currentfigure");
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
    img=apGet(apRobot,"img");
    particles=apGet(apRobot,"particles");
    [l,c]=size(particles);
    dat= datestr (now());
    if (!exist("head1")) 
      head1=([" particles ="]);
    endif
    head=strcat(head1,dat);
    title (head);
    hold on;
    imshow(img,[])
    [a,b]=size(img);
    axis([1,b,1,a],"on","xy");
#    for i=1:l
    for i=1:(round(l/plotRatio)-1)      
      plot(particles(i,1)+shitfCartoX,particles(i,2)+shitfCartoY)
    end
