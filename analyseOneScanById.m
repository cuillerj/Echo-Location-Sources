function p = analyseOneScanById(Id)
%createMatrixAnalyseById(Id)
plotEchoOneScanById(Id)
nbPasRotation=getNbStepsRotation();
valAngle=180/(nbPasRotation-1);
load all_theta
load analyseMat.mat
predLoc = predictOneVsAll(all_theta, analyseMat)
c = floor(predLoc/(nbPasRotation+1));
Maille=c+1
Angle=((predLoc-c*(nbPasRotation)-1))*valAngle
load 'extScanResult.mat'
[x,y]=find(extScanResult(:,5)==predLoc);
extScanResult(x:x+14,:);
plotEchoTraining(predLoc)