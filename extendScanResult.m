function extendScanResult()
% This function extend the file containing the original mesurments in order to add the angular attribut
% It adds x (nbPasRotation) scan that are obtained by shifting the original mesurments
nbPasRotation=getNbStepsRotation()  % number of steps to do 180° rotations 
load ('scanResult.txt'); % the original mesurments that contains IdScan, Angle, Front distance, Back distance and location

shiftIdScan=10000;  % use to translate new records ScanId
nbTrain=size(scanResult,1)  % number of original training records
nbScanId=nbTrain/nbPasRotation  % number of ScanId
extScanResult=zeros((nbTrain*(2*nbPasRotation-3)),5); % will contain the extend data
trainNumber=1;
maxEchoDistance=500; % > maximum distance that the sonar can mesure 

j=1;
n=0;
l=0;
while(j <= nbTrain+1-nbPasRotation) % loop untill end of records
	k=0;
	scanWork=scanResult(j:j+14,:); %

	while (k<(2*nbPasRotation-3))  % loop for each ScanID
		i=0; 

		while (i<(nbPasRotation-1)) % move upward front and back distances
			n=n+1;
			extScanResult(n,:)=[scanResult(j+l,1)+shiftIdScan,scanResult(j+i,2),scanWork(i+2,3),scanWork(i+2,4),scanResult(j+i,5)*(2*(nbPasRotation)-2)+k+1-(2*nbPasRotation-3)];
			i=i+1;
		end
		n=n+1;

		extScanResult(n,:)=[scanResult(j+l,1)+shiftIdScan,scanResult(j+i,2),scanWork(2,4),scanWork(2,3),scanResult(j+i,5)*(2*(nbPasRotation)-2)+k+1-(2*nbPasRotation-3)];

		i=i+1;

		scanWork=extScanResult(n-14:n,:); % save data from previus cycle
		shiftIdScan=shiftIdScan+1; 
		k=k+1;
		l=mod(k,nbPasRotation);
	end

	j=j+nbPasRotation;
end
%tempScanResult=[scanResult(:,1:4),(scanResult(:,5)*(2*nbPasRotation)-(2*nbPasRotation)+1]; % replace orginal location number
tempScanResult=[scanResult(:,1:4),((scanResult(:,5))*(2*nbPasRotation-2)-(2*nbPasRotation-3))]; % replace orginal location number
extScanResult = [tempScanResult;extScanResult] % concatenate original and extended file
%idx=extScanResult(:,3)==0;
%extScanResult(idx,3)=maxEchoDistance;  % replace 0 by maximum echo distance
%idx=extScanResult(:,4)==0;
%extScanResult(idx,4)=maxEchoDistance; % replace 0 by maximum echo distance
size(extScanResult)
save  ("-mat4-binary","training/extScanResult.mat","extScanResult")