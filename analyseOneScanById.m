function p = analyseOneScanById(Id)
%createMatrixAnalyseById(Id)
plotEchoOneScanById(Id) % plot data to be analysed and create matrix analyseMat
nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
valAngle=180/(nbPasRotation-1); % value of the angle of a step 
load all_theta % load matrix learnt during the traing phase
load analyseMat.mat % load data to be analysed
predLoc = predictOneVsAll(all_theta, analyseMat) % predict loaclization value zone/angle
c = floor((predLoc-1)/(2*nbPasRotation-2))
Maille=c+1 % predicted zone 
reste=predLoc-c*(2*nbPasRotation-2);
Angle=mod(360-(reste-1)*valAngle,360) % predicted angle
load 'extScanResult.mat' % load the training matrix
[x,y]=find(extScanResult(:,5)==predLoc);
extScanResult(x:x+14,:);
plotEchoTraining(predLoc) % plot training data that fit with the prediction