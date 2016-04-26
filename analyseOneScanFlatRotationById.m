function p = analyseOneScanFlatRotationById(Id,rotation)
%createMatrixAnalyseById(Id)
load ("zonesXY.txt");
plotEchoLastScanRotation(Id,1,rotation) % create matrix analyseMat and plot data to be analysed 
nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
%valAngle=180/(nbPasRotation-1); % value of the angle of a step 
load all_thetaFlat % load matrix learnt during the traing phase
load analyseMatRotated.mat % load data to be analysed
nbPred=5  % number of predictions to return by desc probability order
predMat = predictxVsAll(all_thetaFlat, analyseMatRotated,nbPred) % predict loaclization value zone/angle
i=1;
while (i<=nbPred)
	predLoc=predMat(i,1)
	Maille=predLoc % predicted zone 
	zonesXY(Maille,:)  % mapping zones with x y coordinates
	load 'scanResult.txt' % load the training matrix
	[x,y]=find(scanResult(:,5)==predLoc);
	scanResult(x:x+14,:);
	plotEchoTraining(predLoc) % plot training data that fit with the prediction
	i=i+1
end
save  ("-mat4-binary","predMat.mat","predMat")