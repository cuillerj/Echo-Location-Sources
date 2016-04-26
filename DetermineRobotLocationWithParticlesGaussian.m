function [detX,detY,detH,particles] = DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn,particles)
%	sigma=[5,5,10];     % sigma for X, Y, angle
	sigma=[12,12];   
%	load particles
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
	[w,idx]=max(particles(:,4));
	detX=particles(idx,1);
	detY=particles(idx,2);
	detH=particles(idx,3);
	%{
	if (plotOn)
		figure();
		title ("determined particles");
		hold on;
		for i=1:x
			plot(particles(i,1),particles(i,2))
		end
		hold off;
	endif
%}
%	save ("-mat4-binary","particles.mat","particles")
%	particles 
endfunction