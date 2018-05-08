function [] = TfPlot360Features(features)
  [a,b]=size(features);
  nbFeatures=(b-1);
  stepSizeDegre=get360StepSizeDegre();  # in°
  stepSize= stepSizeDegre*pi()/180;  
  for j=1:a
    str=strcat('echo scan:',mat2str(features(j,b)));
    figure();
    title (str);
    hold on;
    axis("square")
    lim=getSensorDistanceLimit();
    xlim([-lim,lim]);
    ylim([-lim,lim]);

    angleRadian=-pi()/2;
    i=1;
    while (angleRadian<pi()/2)
      x=features(j,i)*cos(angleRadian);
      y=features(j,i)*sin(angleRadian);
      plot(x,y,'*')
      angleRadian=angleRadian+stepSize;
      i=i+1;
    endwhile
    i=1;
    angleRadian=-pi()/2;
    while (angleRadian<pi()/2)
      x=features(j,i+nbFeatures/2)*cos(pi()+angleRadian);
      y=features(j,i+nbFeatures/2)*sin(pi()+angleRadian);
      plot(x,y,'*')
      angleRadian=angleRadian+stepSize;
      i=i+1;
    endwhile
    
    hold off
 endfor
endfunction