function [] = ToolPlotAnalyyseTrajectory(apRobot)
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  load traceEcho
  load "actual.txt"
  load carto1img;
  load traceDet;
  load traceNext;
  load traceRobot;
  %shitfCartoX=0;
  %shitfCartoY=50;
  img=carto1img;
  trajectories=figure()
  title ("trajectories");
  [a,b]=size(img);
  hold on;
  grid minor on ;
  imshow(img,[]);
  axis([1,b+20,1,a+20],"on","xy");
  text(a-10,b-320,"actual","color",'r');
  text(a-10,b-330,"theoritical","color",'g');
  text(a-10,b-340,"robot","color",'b');
  text(a-10,b-350,"gyroscope","color",'m');
  text(a-10,b-360,"determined","color",'k');
  for i=1:size(actual,1)
    %plot(actual(i,1),a+1-actual(i,2),'r','marker','o');
    plot(actual(i,1),actual(i,2)+shitfCartoY,'r','marker','o','markersize',12);
  end
  for i=1:size(traceNext,1)
    %plot(traceEcho(i,3),a+1-traceEcho(i,6),'g','marker','*');
      plot(traceNext(i,3),traceNext(i,4)+shitfCartoY,'g','marker','*','markersize',12);
  end
  for i=1:size(traceRobot,1)
  %	plot(traceEcho(i,4),a+1-traceEcho(i,7),'b','marker','+');
      plot(traceRobot(i,4),traceRobot(i,5)+shitfCartoY,'b','marker','+','markersize',12);
  end
  for i=1:size(traceEcho,1)
    %plot(traceEcho(i,5),a+1-traceEcho(i,8),'m','marker','s');
  %		plot(traceEcho(i,3),traceEcho(i,4)+shitfCartoY,'m','marker','s','markersize',10);
  end
  for i=1:size(traceDet,1)
    %plot(traceDet(i,3),a+1-traceDet(i,4),'k','markersize',10);
      plot(traceDet(i,3),traceDet(i,4)+shitfCartoY,'k','markersize',20);
  end

  hold off;

endfunction