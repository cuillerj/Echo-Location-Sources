function [detX,detY,detH,particles] = DetermineRobotLocationWithParticlesGaussian(inX,inY,inProb,img,plotOn,particles)
%	sigma=[5,5,10];     % sigma for X, Y, angle
%	sigma=[1,1]*sqrt(inX(1)^2+inY(1)^2);   
	sigma=[15,15];  
%	sigma=[0.1,0.1];  
%	load particles
	inProb=inProb/sum(inProb); % normalyze 
	[x,y]=size(particles);
	z=size(inX,2);
	for i=1:x
		weight=[];
		W=[particles(i,1:2)];      % get(x,y) of particles
		for j=1:z
			X=[inX(j),inY(j)]; % get measurment (x,y)
%			weight=[weight;[normpdf(X,W,sigma)]*inProb(j)+particles(i,4)];           % compute gaussian normal
			weight=[weight;[normpdf(W,X,sigma)]*inProb(j)];           % compute gaussian normal
		endfor
%		[k,l]=max(sum(weight,2))     % sum the weight for each measurment and keep the highest value corresponding to the better choice 
		[k,l]=max(prod(weight,2));     % sum the weight for each measurment and keep the highest value corresponding to the better choice 
%		particles(i,4)=k*(inProb(l)); % update particle weight taking into account the measurment probability
		particles(i,4)=k;
	endfor
	particles(:,4)=particles(:,4)/sum(particles(:,4));  % normalyze the weight to get probability
	[w,idx]=max(particles(:,4));
	detX=particles(idx,1);
	detY=particles(idx,2);
	detH=particles(idx,3);
	if (plotOn)
		figure();
		title ("determined particles");
		hold on;
		imshow(img,[])
		[a,b]=size(img);
		for i=1:x
			plot(particles(i,1),a+1-particles(i,2))
		end
			plot(detX,a+1-detY,"color","r","+","markersize",15)
		for j=1:z
			if (j==1)
				plot(inX(j),a+1-inY(j),"color","g","o","markersize",15)
			else
				plot(inX(j),a+1-inY(j),"color","c","s","markersize",15)
			endif
		endfor
		hold off;
	endif

%	save ("-mat4-binary","particles.mat","particles")
%	particles 
endfunction