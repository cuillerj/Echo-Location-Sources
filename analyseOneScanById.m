function p = analyseOneScanById(Id)
%createMatrixAnalyseById(Id)
plotEchoOneScanById(Id)
nbPasRotation=getNbStepsRotation();
valAngle=180/(nbPasRotation-1);
load all_theta
load analyseMat.mat
predLoc = predictOneVsAll(all_theta, analyseMat)
c = floor((predLoc-1)/(2*nbPasRotation-2))
Maille=c+1
reste=predLoc-c*(2*nbPasRotation-2);
Angle=mod(360-(reste-1)*valAngle,360)
load 'extScanResult.mat'
[x,y]=find(extScanResult(:,5)==predLoc);
extScanResult(x:x+14,:);
plotEchoTraining(predLoc)