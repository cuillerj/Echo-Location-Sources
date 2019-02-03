
function [apRobot,robot,detX,detY,detH,prob,figureNumber] = ApDetermineRobotLocationWithTfAndParticlesGaussian(apRobot,robot,inX,inY,inProb,plotOn,newFigure)
   printf(mfilename);
   printf(" ***  ");
   printf(ctime(time()));	
   probCum=0;
   figureNumber=0;
	img=apGet(apRobot,"img");
  plotRatio=3; # 1/plotRatio point will be plotted
	particles=apGet(apRobot,"particles");
	sigma=[40];  
	inProb=inProb/sum(inProb); % normalyze 
	[x,y]=size(particles);
	z=size(inX,2);
  maxPredictionToUse=5;  % over this value prediction is no longer effective
  minProbToTakeIntoAccount=0.05;
  if (!exist("newFigure"))  % rese or not figure(2)
      newFigure=false;
  endif
  #{
	for i=1:x
		weight=[];
		W=[particles(i,1:2)];      % get(x,y) of particles
		for j=1:z
			X=[inX(j),inY(j)]; % get measurment (x,y)
			weight=[weight;[normpdf(W,X,sigma)]*inProb(j)];           % compute gaussian normal
		endfor
	%	[k,l]=max(prod(weight,2));     % sum the weight for each measurment and keep the highest value corresponding to the better choice 
		k=sum(prod(weight,2)); 
    particles(i,4)=k;
	endfor
  #}
  for i=1:x
      density=0;
      for j=1:min(z,maxPredictionToUse)
           if (inProb(j)>=minProbToTakeIntoAccount)
             dist=sqrt((particles(i,1)-inX(j))^2+(particles(i,2)-inY(j))^2);
             density=density+normpdf(dist,0,sigma)*100*inProb(j);
           endif
      endfor
      particles(i,4)=particles(i,4)*density;
  endfor  
	particles(:,4)=particles(:,4)/sum(particles(:,4));  % normalyze the weight to get probability
	[prob,idx]=max(particles(:,4));
	detX=round(particles(idx,1));
	detY=round(particles(idx,2));
	detH=round(mod(particles(idx,3),360));
  detHG=detH*pi()/180;
  forward=apGet(apRobot,"forward");
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  orig=[detX+shitfCartoX,detY+shitfCartoY];
  if (forward)
    tip=[orig-[15*cos(detHG),15*sin(detHG)]];
    arrows = [tip orig];
  else
    tip=[orig+[15*cos(detHG),15*sin(detHG)]];
    arrows = [orig tip];
  endif
  apRobot = setfield(apRobot,"particles",particles);
  determine=5;
  checkLocation=8;
  targeting=3;
  if (apGet(apRobot,("automatonState"))(1)==targeting)
    action=checkLocation;
  else
    action=determine;
  endif
  [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[action,0],1);
 if (plotOn)
  figure(1);
  hold on;
  drawArrowJC (arrows, 5, 1, 0.5, 1);
	plot(detX+shitfCartoX,detY+shitfCartoY,"color","b","+","markersize",15);
 endif
	if (plotOn)
    %{
    if(newFigure)
      carto=apGet(apRobot,"carto");
      img=apGet(apRobot,"img");
      figure();    % f
    %  title (" determined location(+)");
      hold on;
      imshow(img,[]);       % insert map background
      [a,b]=size(img);
      c=max(a,b);
      axis([1,b,1,a],"on","xy");
    else
      figure(2);
    endif
    %}
	%	hold on;
 %   figureNumber=get (0, "currentfigure");
    [apRobot,figureNumber] = ApPlotParticles(apRobot,plotRatio," determined location(+) >");
    printf(mfilename);
    printf(" figure :%d ***  ",figureNumber);
    printf(ctime(time()));	
    %{
		for i=1:(round(x/plotRatio)-1)
			plot(particles(plotRatio*i,1)+shitfCartoX,particles(plotRatio*i,2)+shitfCartoY)
		end
    %}
			plot(detX+shitfCartoX,detY+shitfCartoY,"color","g","+","markersize",20)
		for j=1:min(z,maxPredictionToUse)
			if (j==1)
				plot(inX(j)+shitfCartoX,inY(j)+shitfCartoY,"color","k","o","markersize",15)
			else
        if (inProb(j)>=minProbToTakeIntoAccount)
				  plot(inX(j)+shitfCartoX,inY(j)+shitfCartoY,"color","k","s","markersize",15)
        endif
			endif
		endfor
    grid minor;
	%	hold off;
  %}
	endif
   printf(mfilename);
   printf(" Determined location:(%d,%d,%d) probability:%.1f%% ***  ",detX,detY,detH,100*prob);
   printf(ctime(time()));
endfunction