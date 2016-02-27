function p = evaluateLearningResults()
load ("zonesXY.txt");
load ("validationSample.txt");
load accuracy;
nbRec=size(validationSample,1);
load 'extScanResult.mat'; % load the training matrix
for j=(1:nbRec)
	plotEchoOneScanById(mat2str(validationSample(j,1)),0) % plot data to be analysed and create matrix analyseMat
	nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
	valAngle=180/(nbPasRotation-1); % value of the angle of a step 
	load all_theta; % load matrix learnt during the traing phase
	load analyseMat.mat; % load data to be analysed
	nbPred=5;  % number of predictions to return by desc probability order
	predMat = predictxVsAll(all_theta, analyseMat,nbPred); % predict loaclization value zone/angle
	i=1;
	while (i<=nbPred)
		predLoc=predMat(i,1);
		predValue=predMat(i,2);
		c = floor((predLoc-1)/(2*nbPasRotation-2));
		Maille=c+1; % predicted zone 
		zonesXY(Maille,:);  % mapping zones with x y coordinates
		reste=predLoc-c*(2*nbPasRotation-2);
		Angle=mod((reste-1)*valAngle,360); % predicted angle
		[x,y]=find(extScanResult(:,5)==predLoc);
		evaluationMatrix((j-1)*nbPred+i,1)=(validationSample(j,1));
		evaluationMatrix((j-1)*nbPred+i,2)=(zonesXY(Maille,1));
		evaluationMatrix((j-1)*nbPred+i,3)=(zonesXY(Maille,2));
		evaluationMatrix((j-1)*nbPred+i,4)=(Angle);
		evaluationMatrix((j-1)*nbPred+i,5)=predValue;
		evaluationMatrix((j-1)*nbPred+i,6)=zonesXY(validationSample(j,2),1);
		evaluationMatrix((j-1)*nbPred+i,7)=zonesXY(validationSample(j,2),2);
		evaluationMatrix((j-1)*nbPred+i,8)=validationSample(j,3);
		evaluationMatrix((j-1)*nbPred+i,9)=i;
%		extScanResult(x:x+14,:);
		i=i+1;
	end
endfor
save  ("-mat4-binary","evaluationMatrix.mat","evaluationMatrix")
load evaluationMatrix;
nbRec=size(evaluationMatrix,1);
coeff=0;
foundFirst=0;
notFound=size(validationSample,1);
prevFound=0;
for j=(1:nbRec)
	if (abs(evaluationMatrix(j,2)-evaluationMatrix(j,6))<=16 && abs(evaluationMatrix(j,3)-evaluationMatrix(j,7))<=16 && (abs(evaluationMatrix(j,4)-evaluationMatrix(j,8))<=15 || abs(evaluationMatrix(j,4)-evaluationMatrix(j,8))>=345))
	coeff=coeff+100/evaluationMatrix(j,9);
	if (prevFound!=evaluationMatrix(j,1))
		prevFound=evaluationMatrix(j,1);
		notFound=notFound-1;
	endif
	if (evaluationMatrix(j,9)==1)
		foundFirst=foundFirst+1;
	endif
	printf("found for:%d sol value:%d pred value:%d . \n",evaluationMatrix(j,1),100/evaluationMatrix(j,9),evaluationMatrix(j,5))
	endif
endfor
	printf("value:%d found fisrt:%d not found:%d. \n",coeff,foundFirst,notFound)
