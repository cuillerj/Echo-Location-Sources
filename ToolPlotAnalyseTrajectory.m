function [] = ToolPlotAnalyseTrajectory(apRobot)
  %{
  This tool plot the different vues of trajectory
  from traceDet the determined trajectory
  from traceNext the theoritical trajectory
  from traceRobot the robot estimaed trajectory
  from actual.txt the actual trajectory that has to be documented manualy
  %}
  
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  load "actual.txt"
  load carto1img;
  load traceDet;
  load traceNext;
  load traceRobot;
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
  text(a-10,b-360,"determined","color",'k');
  for i=1:size(actual,1)
    plot(actual(i,1),actual(i,2)+shitfCartoY,'r','marker','o','markersize',20);
  end
  for i=1:size(traceNext,1)

      plot(traceNext(i,3),traceNext(i,4)+shitfCartoY,'g','marker','*','markersize',20);
  end
  for i=1:size(traceRobot,1)
      plot(traceRobot(i,4),traceRobot(i,5)+shitfCartoY,'b','marker','+','markersize',20);
  end
  for i=1:size(traceDet,1)
      plot(traceDet(i,3),traceDet(i,4)+shitfCartoY,'k','marker','x','markersize',25);
  end

  hold off;

endfunction