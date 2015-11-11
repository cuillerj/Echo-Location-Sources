function [] = plotEchoOneScanById(Id)
createMatrixAnalyseById(Id)
load ('analyseMat.mat');
% rec correspond a la combinaison case angle

 angle=linspace(1,181,181);
 col=['r','g','b','m','c'];
figure();
title (Id);
hold on;

 Z=reshape(analyseMat(1,:),2,181);
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