function [locX,locY,locAngle,locCost] = analyseLastScanRotation(robot,plotOn,nbPred,rotation)
%createMatrixAnalyseById(Id)
%javamethods(robot)
%robot.GetLastScanID()
%nbPred=5  % number of predictions to return by desc probability order
locCost=-1;
locCost=0; 
locAngle=0;
load ("zonesXY.txt");

plotEchoLastScanRotation(robot,plotOn,rotation) % plot data to be analysed and create matùrix analyseMat
nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
%valAngle=180/(nbPasRotation-1); % value of the angle of a step 
load all_thetaFlat % load matrix learnt during the traing phase
load analyseMatRotated.mat % load data to be analysed
i=1;

%predLoc = predictOneVsAll(all_theta, analyseMat) % predict loaclization value zone/angle
predMat = predictxVsAll(all_thetaFlat, analyseMatRotated,nbPred) % predict loaclization value zone/angle
load 'scanResult.txt' % load the training matrix
while (i<=nbPred)
		predLoc=predMat(i,1);
		c = floor((predLoc-1)/(2*nbPasRotation-2));
		Maille=predLoc; % predicted zone 
		zonesXY(Maille,:);
		locX(i)=zonesXY(Maille,1);
		locY(i)=zonesXY(Maille,2);
		locAngle(i)=rotation;
		reste=predLoc-c*(2*nbPasRotation-2);
		locCost(i)=predMat(i,2);
		[x,y]=find(scanResult(:,5)==predLoc);
		scanResult(x:x+14,:);
		if (plotOn==true)
			plotEchoTraining(predLoc) % plot training data that fit with the prediction
		endif
		i=i+1;
end
%p class (p,"analyseLastScan",robot)

