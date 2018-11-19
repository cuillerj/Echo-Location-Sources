function [extScanResult] = TfPartiallyExtendScanResult()
	% This function extend the file containing the original mesurments in order to add the angular attribut
	% It adds nbStepsToAdd virtual scans that are obtained by shifting the original mesurments
  % output is "partiallyExtendedScanResult.csv" 
  %{
    "partiallyExtendedScanResult.csv"  rows: 1:scanid  2:posX 3:posY 4:scanAngle 5:distFront  6:distBack  7:virtualRotation (mulitple of steps rotated)
  %}
  printf(mfilename);
  printf(" start ")
  printf(ctime(time()))
  nbPasRotation=getNbStepsRotation()  % number of steps for a 180° rotation
	#load ('scanFlatResult.csv'); % the original mesurments that contains IdScan, Angle, Front distance, Back distance and location
  load('learningScanFlatData.mat');
  scanResult=flatData;
	shiftIdScan=10000;  % use to translate new records ScanId avoiding conflict with the original Id
	nbTrain=size(scanResult,1)  % number of original training records
	nbScanId=nbTrain/nbPasRotation  % get the number of different IdScan
  stepSize=round(180/(nbPasRotation-1));
	#extScanResult=zeros((nbTrain*(2*nbPasRotation-3)),5); % extScanResult will contain the extend data
  extScanResult=[];
	trainNumber=1;
	maxEchoDistance=getSensorDistanceLimit(); % > maximum distance that the sonar can mesure 
  nbStepsToAdd=2;
	j=1;
	n=0;
	l=0;
	while(j <= nbTrain+1-nbPasRotation) % loop ScanId untill end of records
		k=0;
		scanWork=scanResult(j:j+14,:); % save value that will be needed to rotate
    i=0;
    newRow=zeros(nbPasRotation,1);
    newScan=[scanWork,newRow];
    extScanResult=[extScanResult;[newScan]];
    while(i<nbStepsToAdd)
      x=circshift (scanWork(:,5:6),-1);
      y=x(nbPasRotation,2);
      x(nbPasRotation,2)=x(nbPasRotation,1);
      x(nbPasRotation,1)=y;
      i=i+1;
      scanWork(:,5:6)=x;
      newRow=ones(nbPasRotation,1)*-i*stepSize;
      newScan=[scanWork,newRow];
      extScanResult=[extScanResult;[newScan]];
    endwhile
    scanWork=scanResult(j:j+14,:); % save value that will be needed to rotate
    i=0;
    while(i<nbStepsToAdd)
      x=circshift (scanWork(:,5:6),1);
      y=x(1,2);
      x(1,2)=x(1,1);
      x(1,1)=y;
      i=i+1;
      scanWork(:,5:6)=x;
      newRow=ones(nbPasRotation,1)*i*stepSize;
      newScan=[scanWork,newRow];
      extScanResult=[extScanResult;[newScan]];
    endwhile
		j=j+nbPasRotation; % move to the next ScanID
	end
	#tempScanResult=[scanResult(:,1:4),((scanResult(:,5))*(2*nbPasRotation-2)-(2*nbPasRotation-3))]; % replace orginal location number
	#extScanResult = [tempScanResult;extScanResult]; % concatenate original file and extended file
	#size(extScanResult)
  #labelsNumber=max(extScanResult(:,5))
	csvwrite ("partiallyExtendedScanResult.csv", extScanResult);
endfunction