function [apRobot,robot,detX,detY,detH] = ApDetermineRobotLocationWithParticlesGaussian(apRobot,robot,inX,inY,inProb,plotOn)
	img=apGet(apRobot,"img");
	particles=apGet(apRobot,"particles");
	sigma=[15,15];  
	inProb=inProb/sum(inProb); % normalyze 
	[x,y]=size(particles);
	z=size(inX,2);
	for i=1:x
		weight=[];
		W=[particles(i,1:2)];      % get(x,y) of particles
		for j=1:z
			X=[inX(j),inY(j)]; % get measurment (x,y)
			weight=[weight;[normpdf(W,X,sigma)]*inProb(j)];           % compute gaussian normal
		endfor
		[k,l]=max(prod(weight,2));     % sum the weight for each measurment and keep the highest value corresponding to the better choice 
		particles(i,4)=k;
	endfor
	particles(:,4)=particles(:,4)/sum(particles(:,4));  % normalyze the weight to get probability
	[w,idx]=max(particles(:,4));
	detX=round(particles(idx,1));
	detY=round(particles(idx,2));
	detH=round(mod(particles(idx,3),360));
	if (plotOn)
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
		figure();
		title ("determined particles");
		hold on;
		imshow(img,[])
		[a,b]=size(img);
		axis([1,b,1,a],"on","xy");
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
		hold off;
	endif
endfunction