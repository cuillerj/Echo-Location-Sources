function [] = DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)
%	sigma=[5,5,10];     % sigma for X, Y, angle
	sigma=[12,12];   
	load particles;
	[x,y]=size(particles);
	z=size(echoX,2);
	for i=1:x
		weight=[];
		W=[particles(i,1:2)];       % get(x,y) of particles
		for j=1:z
			X=[echoX(j),echoY(j)]; % get measurment (x,y)
			weight=[weight;[normpdf(X,W,sigma)]];           % compute gaussian normal
		endfor

		Z=max(sum(weight,2));     % sum the weight for each measurment and keep the highest value corresponding to the better choice 
		[k,l]=max(Z);
		particles(i,4)=k*(101-echoProb(l)); % update particle weight taking into account the measurment probability
	endfor
	particles(:,4)=particles(:,4)/sum(particles(:,4));  % normalyze the weight to get probability
	save ("-mat4-binary","particles.mat","particles")
%	particles 
endfunction