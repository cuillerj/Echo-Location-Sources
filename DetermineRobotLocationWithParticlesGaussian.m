function [] = DetermineRobotLocationWithParticlesGaussian(echoX,echoY,echoProb,plotOn)
%	sigma=[5,5,10];     % sigma for X, Y, angle
	sigma=[12,12];   
	load particles;
	[x,y]=size(particles);
	z=size(echoX,2);
	for i=1:x
		weight=[];
%		W=[particles(i,1:2),[180]];
				W=[particles(i,1:2)];
		for j=1:z
%			[a,b]=max(echoAngle(j),particles(i,3:3)];
%%			p2=particles(i,3:3);
%			if (p1-p2<0)
%				p2=echoAngle(j);
%				p1=particles(i,3:3);
%			endif
%			diffAngle=min(p1-p2,p1-p2+360);
%			X=[echoX(j),echoY(j),mod((echoAngle(j)+180-particles(i,3:3)+360),360)];
			X=[echoX(j),echoY(j)];
			weight=[weight;[normpdf(X,W,sigma)]];           % gaussian normal

		endfor
%		weight(:,3)=weight(:,3).*2;
		Z=max(sum(weight,2));
		[k,l]=max(Z);
		particles(i,4)=k*(101-echoProb(l));
%		particles(i,4)=k;
	endfor
	particles(:,4)=particles(:,4)/sum(particles(:,4));  % normalyze
	save ("-mat4-binary","particles.mat","particles")
%	particles 
endfunction