function [coeff,foundFirst,notFound,nbrec] = evaluateLearningResultsFlat()
load ("zonesXY.txt");
load ("validationSample.txt");
load accuracyFlat;
nbRec=size(validationSample,1);
idx=validationSample(:,3)==0;  % to select only the 0° orientation
nbrec=sum(idx);
load 'scanResult.txt'; % load the training matrix
num=0;
for j=(1:nbRec)

	if (idx(j)==1)
		num=num+1;
		plotEchoOneScanById(mat2str(validationSample(j,1)),0) % plot data to be analysed and create matrix analyseMat
%		nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
%		valAngle=180/(nbPasRotation-1); % value of the angle of a step 
		load all_thetaFlat; % load matrix learnt during the traing phase
		load analyseMat.mat; % load data to be analysed
		nbPred=5;  % number of predictions to return by desc probability order
		predMat = predictxVsAll(all_thetaFlat, analyseMat,nbPred); % predict loaclization value zone/angle
		i=1;
		while (i<=nbPred)
				predLoc=predMat(i,1);
				predValue=predMat(i,2);
				Maille=predLoc; % predicted zone 
				zonesXY(Maille,:);  % mapping zones with x y coordinates
				[x,y]=find(scanResult(:,5)==predLoc);
				evaluationMatrixFlat((num-1)*nbPred+i,1)=(validationSample(j,1));
				evaluationMatrixFlat((num-1)*nbPred+i,2)=(zonesXY(Maille,1));
				evaluationMatrixFlat((num-1)*nbPred+i,3)=(zonesXY(Maille,2));
				evaluationMatrixFlat((num-1)*nbPred+i,4)=predValue;
				evaluationMatrixFlat((num-1)*nbPred+i,5)=zonesXY(validationSample(j,2),1);
				evaluationMatrixFlat((num-1)*nbPred+i,6)=zonesXY(validationSample(j,2),2);
				evaluationMatrixFlat((num-1)*nbPred+i,7)=i;
	%		scanResult(x:x+14,:);
			i=i+1;
		end
	endif
endfor
save  ("-mat4-binary","evaluationMatrixFlat.mat","evaluationMatrixFlat")
load evaluationMatrixFlat;
nbRec=size(evaluationMatrixFlat,1);
coeff=0;
foundFirst=0;
notFound=nbrec;
prevFound=0;
for j=(1:nbRec)
	if (abs(evaluationMatrixFlat(j,2)-evaluationMatrixFlat(j,5))<=16 && abs(evaluationMatrixFlat(j,3)-evaluationMatrixFlat(j,6))<=16 )
	coeff=coeff+100/evaluationMatrixFlat(j,7);
	if (prevFound!=evaluationMatrixFlat(j,1))
		prevFound=evaluationMatrixFlat(j,1);
		notFound=notFound-1;
	endif
	if (evaluationMatrixFlat(j,7)==1)
		foundFirst=foundFirst+1;
	endif
	printf("found for:%d sol value:%d pred value:%d . \n",evaluationMatrixFlat(j,1),100/evaluationMatrixFlat(j,7),evaluationMatrixFlat(j,5))
	endif
endfor
	printf("value:%d found fisrt:%d not found:%d. total records:%d \n",coeff,foundFirst,notFound,nbrec)
