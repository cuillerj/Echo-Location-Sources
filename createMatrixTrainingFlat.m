% create a 360° matrix trainMatFlat 
% the original data are extrapolated to provide distances for each degree
% 
nbMesurementByTrain=getNbStepsRotation(); % get the number of steps for a 180 rotation
load ('scanResult.txt');
nbTrain=size(scanResult,1)
pixelBF=zeros(2,181);
trainMatFlat=zeros((nbTrain/nbMesurementByTrain),362);
trainResultFlat=zeros((nbTrain/nbMesurementByTrain),1);
trainNumber=1;
j=1;
i=1;
while(j < nbTrain)  % loop untill the end of data
	trainNumber;
	pixelBF=zeros(2,181);
	idScan=scanResult(j,1);  % get the ScanID
	trainResultFlat(trainNumber)=scanResult(j,5);  % get the zone/angle value
		while (scanResult(j,1)<=idScan && j < nbTrain)  % loop for each ScanID
				scanResult(j,2);  
				angle=scanResult(j,2); % get the angle value
				k=angle;
				pixelBF(1,k+1)=scanResult(j,3); % get the front distance value
				pixelBF(2,k+1)=scanResult(j,4); % get the back distance value

				if (j < nbTrain)
					angleNext=scanResult(j+1,2); % get the next angle value
					interval=1;
						while (k < angleNext )  % loop until the next angle is reached
							% extrapolate distances between 2 angles in a linear way
							% create a value for each degree
							pixelBF(1,k+2)=scanResult(j,3)+(scanResult(j+1,3)-scanResult(j,3))/(angleNext-angle)*interval; % 
							pixelBF(2,k+2)=scanResult(j,4)+(scanResult(j+1,4)-scanResult(j,4))/(angleNext-angle)*interval;
							% o distance value is not very significant 
							if (scanResult(j,3)==0 || scanResult(j+1,3)==0)  % no extrapolation if distance is 0
								pixelBF(1,k+2)=0;
							end
							if (scanResult(j,4)==0 || scanResult(j+1,4)==0) % no extrapolation if distance is 0
								pixelBF(2,k+2)=0;
							end
							k=k+1;
							interval=interval+1;
						end
				end
				j=j+1  ;
		end
		pixelBF;
		trainMatFlat(trainNumber,:)=reshape (pixelBF,1,[]);
		i=i+1;
		trainNumber=trainNumber+1; 

end
size(trainMatFlat)
save  ("-mat4-binary","training/trainMatFlat.mat","trainMatFlat")
save  ("-mat4-binary","training/trainResultFlat.mat","trainResultFlat")