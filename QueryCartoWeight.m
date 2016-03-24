function [cartoWeight] = QueryCartoWeight(heading,idX,idY,cartoId,plotOn)
% multiple cartoId to be developped
% taking into account robot size and rotation to be developped
RobotHalfWidth = 25;  % en term of carto cases 1cm/cm
RobotFrontLenght = 40;
RobotBackLenght = 20;
maxWeight=10^6;
securityWeight=100;
load carto1;
[a,b]=size(carto1)
load carto1img;
cartoWeight=0;
%carto2Weight=0;
X=idX;
cosH=cos(heading*pi/180);
sinH=sin(heading*pi/180);
RobotFrontLenghtSecur = 60;
RobotBackLenghtSecur = 25;
RobotHalfWidthSecur=30;
points=[[floor(idX+RobotFrontLenght*cosH),floor(idY+RobotFrontLenght*sinH)],
		[floor(idX+RobotFrontLenght*cosH+RobotHalfWidth*sinH),floor(idY+RobotFrontLenght*sinH-RobotHalfWidth*cosH)],
		[floor(idX+RobotFrontLenght*cosH-RobotHalfWidth*sinH),floor(idY+RobotFrontLenght*sinH+RobotHalfWidth*cosH)],
		[floor(idX-RobotBackLenght*cosH+RobotHalfWidth*sinH),floor(idY-RobotBackLenght*sinH-RobotHalfWidth*cosH)],
		[floor(idX-RobotBackLenght*cosH-RobotHalfWidth*sinH),floor(idY-RobotBackLenght*sinH+RobotHalfWidth*cosH)],
		[floor(idX+RobotFrontLenghtSecur*cosH),floor(idY+RobotFrontLenghtSecur*sinH)],
		[floor(idX+RobotFrontLenghtSecur*cosH+RobotHalfWidthSecur*sinH),floor(idY+RobotFrontLenghtSecur*sinH-RobotHalfWidthSecur*cosH)],
		[floor(idX+RobotFrontLenghtSecur*cosH-RobotHalfWidthSecur*sinH),floor(idY+RobotFrontLenghtSecur*sinH+RobotHalfWidthSecur*cosH)],
		[floor(idX-RobotBackLenghtSecur*cosH+RobotHalfWidthSecur*sinH),floor(idY-RobotBackLenghtSecur*sinH-RobotHalfWidthSecur*cosH)],
		[floor(idX-RobotBackLenghtSecur*cosH-RobotHalfWidthSecur*sinH),floor(idY-RobotBackLenghtSecur*sinH+RobotHalfWidthSecur*cosH)]]
if (plotOn)
%	load carto1img;
	figure()
	hold on;

%	im=imread("cartoimg1.PNG");
%	imagesc(im);
%	imshow(carto1img);
%	imshow (carto1img,[],"xdata",[0,2*a],"ydata",[0,2*b]);
	title ("robot position");
	xlabel("X");
	ylabel("Y");
	xlim([0,a]);
	ylim([0,b]);
	for i=1:5
	plot(points(i,1),points(i,2),'b');
	end
	for i=6:10
	plot(points(i,1),points(i,2),'g');
	end
endif
cartoWeight=0;
for i=1:10
	if (points(i,1)<=a && points(i,1)>0 && points(i,2)<=b && points(i,2)>0 )
		w=carto1(points(i,1),points(i,2));
			if (w<=1)
				cartoWeight=cartoWeight+w;
			else
				if (i<=5)
					printf("costly x: %d. y: %d \n",points(i,1),points(i,2));
					if (plotOn)
%						figure()
						hold on;
						plot(points(i,1),points(i,2),'r',"markersize",10);
					endif
					cartoWeight=cartoWeight+maxWeight;
				else
					cartoWeight=cartoWeight+securityWeight;
				endif
			endif
	endif
hold off;
endfor


endfunction