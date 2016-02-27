function p = analyseOneScanById(Id)
%createMatrixAnalyseById(Id)
load ("zonesXY.txt");
plotEchoOneScanById(Id,1) % plot data to be analysed and create matrix analyseMat
nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
valAngle=180/(nbPasRotation-1); % value of the angle of a step 
load all_theta % load matrix learnt during the traing phase
load analyseMat.mat % load data to be analysed
nbPred=5  % number of predictions to return by desc probability order
predMat = predictxVsAll(all_theta, analyseMat,nbPred) % predict loaclization value zone/angle
i=1;
while (i<=nbPred)
	predLoc=predMat(i,1)
	c = floor((predLoc-1)/(2*nbPasRotation-2));
	Maille=c+1 % predicted zone 
	zonesXY(Maille,:)  % mapping zones with x y coordinates
	reste=predLoc-c*(2*nbPasRotation-2);
	Angle=mod(360-(reste-1)*valAngle,360) % predicted angle

	load 'extScanResult.mat' % load the training matrix
	[x,y]=find(extScanResult(:,5)==predLoc);
	extScanResult(x:x+14,:);
	plotEchoTraining(predLoc) % plot training data that fit with the prediction
	i=i+1
end
