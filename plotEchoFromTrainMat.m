function [] = plotEchoFromTrainMat(rec)
str=strcat('echo choosen:',mat2str(rec));
% rec correspond a la combinaison case angle
 load 'trainMat.mat';
 angle=linspace(1,181,181);
 col=['r','g','b','m','c'];
figure();
title (str);
hold on;
i=rec
 Z=reshape(trainMat(i,:),2,181);
	dX=Z(1,:);
	dY=Z(2,:);
	angleR=angle*pi()/180;
	X=dX.*cos(angleR);
	Y=dX.*sin(angleR);
	plot (X,Y,col(3));

	X=-dY.*cos(angleR);
	Y=-dY.*sin(angleR);
	plot (X,Y,col(3));

hold off;