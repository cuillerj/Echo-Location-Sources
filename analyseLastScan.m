function [locX,locY,locAngle,locCost] = analyseLastScan(robot,plotOn,nbPred)
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
load all_theta % load matrix learnt during the traing phase
load analyseMat.mat % load data to be analysed
i=1;

%predLoc = predictOneVsAll(all_theta, analyseMat) % predict loaclization value zone/angle
predMat = predictxVsAll(all_theta, analyseMat,nbPred) % predict loaclization value zone/angle
load 'extScanResult.mat' % load the training matrix
while (i<=nbPred)
		predLoc=predMat(i,1)
		c = floor((predLoc-1)/(2*nbPasRotation-2))
		Maille=c+1 % predicted zone 
		zonesXY(Maille,:)
		locX(i)=zonesXY(Maille,1);
		locY(i)=zonesXY(Maille,2);
		reste=predLoc-c*(2*nbPasRotation-2);
		Angle=mod(360-(reste-1)*valAngle,360) % predicted angle
		locAngle(i)=Angle;
		locCost(i)=predMat(i,2);
		[x,y]=find(extScanResult(:,5)==predLoc);
		extScanResult(x:x+14,:);
		if (plotOn==true)
			plotEchoTraining(predLoc) % plot training data that fit with the prediction
		endif
		i=i+1;
end
%p class (p,"analyseLastScan",robot)

