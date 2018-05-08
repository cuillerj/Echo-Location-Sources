#{
 create a 360Â° matrix trainMat 
 the input data are extrapolated to provide distances for each degree
 trainAndTestFeaturesData.csv contains for each scan a flat records of (181*front_distances, 181*back_distances,label number)
 trainAndTestLabelsData.csv contains the corresponding (labelId,posX,posY,shiftIndex)
#}
function [pixelFront,pixelBack,pixelFB,zonesList] = TfCreatePartiallyFlatMatrix()
  incrementValue=get360StepSizeDegre()  # nb of ° incrementation by step 
  nbMesurementByTrain=getNbStepsRotation(); % get the number of steps to add between 2 mesurments (in °)
  load ('partiallyExtendedScanResult.csv');
  extScanResult=partiallyExtendedScanResult;
  nbTrain=size(extScanResult,1)
  ratioTrainTest=15 % 1/x%
  ratioTestValidation=10 % 1/x%
  trainResult=zeros((nbTrain/nbMesurementByTrain)+1,5);  trainNumber=2;
  pixelFB=[];
  nbPasRotation=getNbStepsRotation();  % number of steps for a 180Â° rotation
  j=1;
  flat=[];
  pixelFront=zeros(1,nbMesurementByTrain);
  pixelBack=zeros(1,nbMesurementByTrain);
  pixelIdx=1;
  while(j <= nbTrain)  % loop untill the end of data
    trainResult(trainNumber,:)=[trainNumber-1,extScanResult(j,2:3),extScanResult(j,7),0];  % get the zone/angle value
    trainNumber=trainNumber+1; 
    while ((pixelIdx!=nbPasRotation+1) && j <=nbTrain)  % loop for each ScanID
          pixelFront(1,pixelIdx)=extScanResult(j,5); % get the front distance value
          pixelBack(1,pixelIdx)=extScanResult(j,6); % get the back distance value
          pixelIdx=pixelIdx+1;
         j=j+1  ;
    end
      pixelFB=[pixelFB;[pixelFront,pixelBack,trainNumber-3]];
      pixelIdx=1;
   end
    [l,c]=size(pixelFB)
    
    # create a new list of zones (idx,posX,posY,shift)
    zone=-1;
    zonesList=[zone,[0,0,0]];
    [a,b]=size(trainResult);
    for i=2:a
      checkIn=zonesList(:,2:4)==[trainResult(i,2),trainResult(i,3),trainResult(i,4)];
      [s1,s2]=max(sum(checkIn,2));
      if (s1==3) #found
        idxZone=zonesList(s2,1);
      else
        zone=zone+1;
        idxZone=zone;
        zonesList=[zonesList;[zone,trainResult(i,2),trainResult(i,3),trainResult(i,4)]];        
      endif
    [zlL,zlC]=size(zonesList);
    zonesList(1,1)=zlL-1;
    zonesList(1,2)=zlC;
    TfParameters=[zlL-1,zlC];
    trainResult(i,5)=zonesList(idxZone+2,1);

    endfor
    # replace index by new zone
    [l,c]=size(pixelFB);
    for i=1:l
      pixelFB(i,c)=trainResult(i+1,5);
    endfor
    # radomly separate train and test data
    idxTest=[];
    while (size(idxTest,1)<(l/ratioTrainTest))
      x=randi(round(l));
      if (sum(idxTest==x)==0)
        idxTest= [idxTest;x];
      endif
    endwhile
    testVTensorFlow=[];
    trainTensorFlow=[];
    for i=1:size(idxTest,1)
      testVTensorFlow=[testVTensorFlow;[pixelFB(idxTest(i),:)]];
    endfor
    trainTensorFlow=[];
    for (i=1:l)
      if ((sum(idxTest==i))==0)
        trainTensorFlow=[trainTensorFlow;[pixelFB(i,:)]];
      endif
    endfor
    
    idxValidation=[];
    l1=size(testVTensorFlow,1)
    while (size(idxValidation,1)<l1/ratioTestValidation)
      x=randi(round(l1));
      if (sum(idxValidation==x)==0)
        idxValidation= [idxValidation;x];
      endif
    endwhile 
    testTensorFlow=[];
    validationTensorFlow=[];
    for (i=1:size(idxValidation,1))
      validationTensorFlow=[validationTensorFlow;[testVTensorFlow(idxValidation(i),:)]];
    endfor
    testTensorFlow=[];
    for (i=1:l1)
      if ((sum(idxValidation==i))==0)
        testTensorFlow=[testTensorFlow;[testVTensorFlow(i,:)]];
      endif
    endfor    
    #
    for i=1:c
      header(1,i)=i;  # generates a csv header
    endfor
    [zlL,zlC]=size(trainTensorFlow);
    TfParameters=[TfParameters;[zlC-1,zlL]];
    newPixelFB=[header;pixelFB];
    avgDist=getSensorDistanceLimit()/2;
    trainTensorFlowZeroCentered=trainTensorFlow;
    trainTensorFlowZeroCentered(:,1:size(trainTensorFlowZeroCentered,2)-1)=(trainTensorFlowZeroCentered(:,1:size(trainTensorFlowZeroCentered,2)-1)-avgDist);
    trainTensorFlow=[header;trainTensorFlow];
    trainTensorFlowZeroCentered=[header;trainTensorFlowZeroCentered];
    testTensorFlowZeroCentered=testTensorFlow;
    testTensorFlowZeroCentered(:,1:size(trainTensorFlowZeroCentered,2)-1)=(testTensorFlow(:,1:size(trainTensorFlowZeroCentered,2)-1)-avgDist);
    testTensorFlow=[header;testTensorFlow];
    testTensorFlowZeroCentered=[header;testTensorFlowZeroCentered];
    validationTensorFlowZeroCentered=validationTensorFlow;
    validationTensorFlowZeroCentered(:,1:size(trainTensorFlowZeroCentered,2)-1)=(validationTensorFlow(:,1:size(trainTensorFlowZeroCentered,2)-1)-avgDist);
    validationTensorFlow=[header;validationTensorFlow];
    validationTensorFlowZeroCentered=[header;validationTensorFlowZeroCentered];
    size(trainTensorFlow)
    size(testTensorFlow)
    size(validationTensorFlowZeroCentered)
    csvwrite ("tensorFlow/trainAndTestFeaturesData.csv", newPixelFB);
    csvwrite ("tensorFlow/trainFeaturesData.csv", trainTensorFlow);
    csvwrite ("tensorFlow/testFeaturesData.csv", testTensorFlow);
    csvwrite ("tensorFlow/trainTensorFlowZeroCentered.csv", trainTensorFlowZeroCentered);
    csvwrite ("tensorFlow/testTensorFlowZeroCentered.csv", testTensorFlowZeroCentered);
    csvwrite ("tensorFlow/validationTensorFlowZeroCentered.csv", validationTensorFlowZeroCentered);
    csvwrite ("tensorFlow/zonesList.csv", zonesList);
    csvwrite ("tensorFlow/TfParameters.csv", TfParameters);
endfunction