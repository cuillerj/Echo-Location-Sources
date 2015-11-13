% create a 360° matrix trainMat 
% the original data are extrapolated to provide distances for each degree
% 
nbMesurementByTrain=getNbStepsRotation(); % get the number of steps for a 180 rotation
load ('extScanResult.mat');
nbTrain=size(extScanResult,1)
pixelBF=zeros(2,181);
trainMat=zeros((nbTrain/nbMesurementByTrain),362);
trainResult=zeros((nbTrain/nbMesurementByTrain),1);
trainNumber=1;
j=1;
i=1;
while(j < nbTrain)  % loop untill the end of data
	trainNumber;
	pixelBF=zeros(2,181);
	idScan=extScanResult(j,1);  % get the ScanID
	trainResult(trainNumber)=extScanResult(j,5);  % get the zone/angle value
		while (extScanResult(j,1)<=idScan && j < nbTrain)  % loop for each ScanID
				extScanResult(j,2);  
				angle=extScanResult(j,2); % get the angle value
				k=angle;
				pixelBF(1,k+1)=extScanResult(j,3); % get the front distance value
				pixelBF(2,k+1)=extScanResult(j,4); % get the back distance value

				if (j < nbTrain)
					angleNext=extScanResult(j+1,2); % get the next angle value
					interval=1;
						while (k < angleNext )  % loop until the next angle is reached
							% extrapolate distances between 2 angles in a linear way
							% create a value for each degree
							pixelBF(1,k+2)=extScanResult(j,3)+(extScanResult(j+1,3)-extScanResult(j,3))/(angleNext-angle)*interval; % 
							pixelBF(2,k+2)=extScanResult(j,4)+(extScanResult(j+1,4)-extScanResult(j,4))/(angleNext-angle)*interval;
							% o distance value is not very significant 
							if (extScanResult(j,3)==0 || extScanResult(j+1,3)==0)  % no extrapolation if distance is 0
								pixelBF(1,k+2)=0;
							end
							if (extScanResult(j,4)==0 || extScanResult(j+1,4)==0) % no extrapolation if distance is 0
								pixelBF(2,k+2)=0;
							end
							k=k+1;
							interval=interval+1;
						end
				end
				j=j+1  ;
		end
		pixelBF;
		trainMat(trainNumber,:)=reshape (pixelBF,1,[]);
		i=i+1;
		trainNumber=trainNumber+1; 

end
size(trainMat)
save  ("-mat4-binary","training/trainMat.mat","trainMat")
save  ("-mat4-binary","training/trainResult.mat","trainResult")