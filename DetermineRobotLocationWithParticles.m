function [] = DetermineRobotLocationWithParticles(echoX,echoY,echoAngle,echoProb)
	load particles;
	[x,y]=size(particles);
	z=size(echoAngle,2);
	for i=1:x
		weight=[];
		for j=1:z
			weight=[weight,[(101-echoProb(j))*((sqrt((echoX(j)-particles(i,1))^2+(echoY(j)-particles(i,2))^2))/100+abs((echoAngle(j)-particles(i,3))))]];
		endfor
		particles(i,4)=min(weight);
	endfor
	save ("-mat4-binary","particles.mat","particles")
	particles;
endfunction