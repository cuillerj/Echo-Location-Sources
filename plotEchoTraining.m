function [] = plotEchoTraining(rec)
str=strcat('echo Training location:',mat2str(rec));
% rec correspond a la combinaison case angle soit predLoc
 load 'trainMat.mat';
 load 'trainResult.mat';
 idx=find(trainResult==rec);
 [s1,s2]=size(idx);
 angle=linspace(1,181,181);
 col=['r','g','b','m','c'];
figure();
title (str);
hold on;
for i=1:s1
 Z=reshape(trainMat(idx(i),:),2,181);
	dX=Z(1,:);
	dY=Z(2,:);
	angleR=angle*pi()/180;
	X=dX.*cos(angleR);
	Y=dX.*sin(angleR);
	plot (X,Y,col(mod(i,5)+1));

	X=-dY.*cos(angleR);
	Y=-dY.*sin(angleR);
	plot (X,Y,col(mod(i,5)+1));
 endfor
hold off;