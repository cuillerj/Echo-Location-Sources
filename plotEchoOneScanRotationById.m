function [] = plotEchoOneScanRotationById(Id,plotOn,rotation)
% Id to analyse graph if plotOn=1
createMatrixAnalyseWithRotationById(Id,rotation); % creat matrix
load ('analyseMatRotated.mat'); % load the created matrix
% rec correspond a la combinaison case angle


	angle=linspace(1,181,181); % 
if (plotOn==1)
	col=['r','g','b','m','c']; % graph color 
 
	figure();
	title (Id);
	hold on;
endif

 Z=reshape(analyseMatRotated(1,:),2,181);  % reshape matrix (1 362) > (2 181)
	dX=Z(1,:);
	dY=Z(2,:);
	angleR=angle*pi()/180;
	X=dX.*cos(angleR);
	Y=dX.*sin(angleR);
	if (plotOn==1)
		plot (X,Y,col(3));
	endif
	X=-dY.*cos(angleR);
	Y=-dY.*sin(angleR);
	if (plotOn==1)
		plot (X,Y,col(3));
	endif

if (plotOn==1)
	hold off;
endif