function [apRobot,robot,detX,detY,detH] = ApDetermineRobotLocationWithParticlesGaussian(apRobot,robot,inX,inY,inProb,plotOn,newFigure)
	img=apGet(apRobot,"img");
	particles=apGet(apRobot,"particles");
	sigma=[20,20];  
	inProb=inProb/sum(inProb); % normalyze 
	[x,y]=size(particles);
	z=size(inX,2);
  if (!exist("newFigure"))  % rese or not figure(2)
      newFigure=false;
  endif
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
	particles(:,4)=particles(:,4)/sum(particles(:,4));  % normalyze the weight to get probability
	[w,idx]=max(particles(:,4));
	detX=round(particles(idx,1));
	detY=round(particles(idx,2));
	detH=round(mod(particles(idx,3),360));
  detHG=detH*pi()/180;
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");
  forward=apGet(apRobot,"forward");
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
  [apRobot,robot,newState,retCode] = ApAutomaton(apRobot,robot,[determine,0],1);
 if (plotOn)
  figure(1);
  hold on;
  drawArrowJC (arrows, 5, 1, 0.5, 1);
	plot(detX+shitfCartoX,detY+shitfCartoY,"color","b","+","markersize",15);
 endif
	if (plotOn)
  %  if(newFigure)
      carto=apGet(apRobot,"carto");
      img=apGet(apRobot,"img");
      figure();    % f
      title (" determined location(+)");
      hold on;
      imshow(img,[]);       % insert map background
      [a,b]=size(img);
      c=max(a,b);
      axis([1,b,1,a],"on","xy");
%    else
%      figure(2);
  %  endif
		hold on;
		for i=1:x
			plot(particles(i,1)+shitfCartoX,particles(i,2)+shitfCartoY)
		end
			plot(detX+shitfCartoX,detY+shitfCartoY,"color","k","+","markersize",15)
		for j=1:z
			if (j==1)
				plot(inX(j)+shitfCartoX,inY(j)+shitfCartoY,"color","k","o","markersize",15)
			else
				plot(inX(j)+shitfCartoX,inY(j)+shitfCartoY,"color","k","s","markersize",15)
			endif
		endfor
    grid minor;
	%	hold off;
	endif

endfunction