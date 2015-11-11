function extendScanResult()
% This function extend the file containing the original mesurments in order to add the angular attribut
% It adds x (nbPasRotation) scan that are obtained by shifting the original mesurments
%
%nbMesurementByTrain=15;
nbPasRotation=getNbStepsRotation()  % number of steps to do 180° rotations 
load ('scanResult.txt'); % the original mesurments that contains IdScan, Angle, Front distance, Back distance and location
%scanWork=scanResult;
shiftIdScan=10000;  % use to translate new records ScanId
nbTrain=size(scanResult,1)  % number of original training records
nbScanId=nbTrain/nbPasRotation  % number of ScanId
extScanResult=zeros((nbTrain*(nbPasRotation-1)),5); % will contain the extend data
trainNumber=1;
maxEchoDistance=500; % > maximum distance that the sonar can mesure 
%j=1;
% creation de la liste des scanId
%while(j < nbScanId+1) 
%	j=j+1;
%end
j=1;
n=0;
while(j <= nbTrain) % loop untill end of records
	k=0;
	l=0;
	scanWork=scanResult(j:j+14,:); %
%	saveX2=scanResult(j,3);
%	saveY2=scanResult(j,4);
%	extScanResult(n:n+14,:)=scanResult(j:j+14,:);
	while (k<nbPasRotation-1)  % loop for each ScanID
		i=0; 
%		extScanResult(n:n+14,:)=scanWork;
%		n=n+15;
%		i=1;
		while (i<(nbPasRotation-1)) % move upward front and back distances
			n=n+1;
			extScanResult(n,:)=[scanResult(j+l,1)+shiftIdScan,scanResult(j+i,2),scanWork(i+2,3),scanWork(i+2,4),scanResult(j+i,5)*(nbPasRotation)+k+2-(nbPasRotation)];
			i=i+1;
%			scanWork(i,:)=extScanResult(n,:);
		end
		n=n+1;
%		if (k<14) % insert at the bottom data line 2 from previous file
		extScanResult(n,:)=[scanResult(j+l,1)+shiftIdScan,scanResult(j+i,2),scanWork(2,4),scanWork(2,3),scanResult(j+i,5)*(nbPasRotation)+k+2-(nbPasRotation)];
%		else % at the end insert at the bottom data line 1 from original file
%		extScanResult(n,:)=[scanResult(j+l,1)+shiftIdScan,scanResult(j+i,2),saveY2,saveX2,scanResult(j+i,5)*(nbPasRotation+1)+k+2-(nbPasRotation+1)];
%		end
		i=i+1;
%		if (k>0)
%			extScanResult(n,:)=[scanResult(j+l,1)+shiftIdScan,scanResult(j+i,2),extScanResult((k-1)*nbPasRotation+2,4),extScanResult((k-1)*nbPasRotation+2,3),scanResult(j+i,5)*nbPasRotation+k+1-nbPasRotation];
%		end
%		n=n+1;
%		i=i+1;
		scanWork=extScanResult(n-14:n,:); % save data from previus cycle
		shiftIdScan=shiftIdScan+1; 
		k=k+1;
		l=l+1;
	end

	j=j+nbPasRotation;
end
tempScanResult=[scanResult(:,1:4),scanResult(:,5)*(nbPasRotation)-(nbPasRotation)+1]; % replace orginal location number
extScanResult = [tempScanResult;extScanResult]; % concatenate original and extended file
%idx=extScanResult(:,3)==0;
%extScanResult(idx,3)=maxEchoDistance;  % replace 0 by maximum echo distance
%idx=extScanResult(:,4)==0;
%extScanResult(idx,4)=maxEchoDistance; % replace 0 by maximum echo distance
size(extScanResult)
save  ("-mat4-binary","training/extScanResult.mat","extScanResult")