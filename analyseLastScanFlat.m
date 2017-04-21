function [locX,locY,locCost] = analyseLastScanFlat(robot,plotOn,nbPred)
%createMatrixAnalyseById(Id)
%javamethods(robot)
%robot.GetLastScanID()
%nbPred=5  % number of predictions to return by desc probability order
locCost=-1;
locCost=0; 
locAngle=0;
load ("zonesXY.txt");

plotEchoLastScan(robot,plotOn) % plot data to be analysed and create matùrix analyseMat
nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
valAngle=180/(nbPasRotation-1); % value of the angle of a step 
load all_thetaFlat; % load matrix learnt during the traing phase
load analyseMat.mat; % load data to be analysed
i=1;

%predLoc = predictOneVsAll(all_theta, analyseMat) % predict loaclization value zone/angle
predMat = predictxVsAll(all_thetaFlat, analyseMat,nbPred) % predict loaclization value zone/angle
load 'scanResult.mat' % load the training matrix
while (i<=nbPred)
		predLoc=predMat(i,1)
		c = floor((predLoc-1)/(2*nbPasRotation-2))
		Maille=predLoc % predicted zone 
%		Maille=predLoc % predicted zone 
		zonesXY(Maille,:)
		locX(i)=zonesXY(Maille,1);
		locY(i)=zonesXY(Maille,2);
		reste=predLoc-c*(2*nbPasRotation-2);
		locCost(i)=predMat(i,2);
		[x,y]=find(scanResult(:,5)==predLoc);
		scanResult(x:x+14,:);
		if (plotOn==true)
			plotEchoFlatTraining(predLoc) % plot training data that fit with the prediction
		endif
		i=i+1;
end
%p class (p,"analyseLastScan",robot)

