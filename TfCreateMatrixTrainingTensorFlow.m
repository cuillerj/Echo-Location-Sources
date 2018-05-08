% create a flat° matrix trainMatFlat 
% the original data are extrapolated to provide distances for each degree
% 
function [trainTensorFlow,testTensorFlow,tensorFlow,idxTest] = TfCreateMatrixTrainingTensorFlow()
  ratioTrainTest=10 % 1/x%
  nbMesurementByTrain=getNbStepsRotation(); % get the number of steps for a 180 rotation
  #load ('scanResult.txt');
  scanResult=load ('tensorFlow/extScanResult.csv');
  nbTrain=size(scanResult,1);
  printf("training records number:%d,number of mesurment by train:%d scan nuùber:%d \n",nbTrain,nbMesurementByTrain,nbTrain/nbMesurementByTrain)
  tensorFlow=[];
  for i=0:nbTrain/nbMesurementByTrain-1
    scanResult(i*nbMesurementByTrain+1:i*nbMesurementByTrain+15,3:4);
    tensorFlow=[tensorFlow;[reshape(scanResult(i*nbMesurementByTrain+1:i*nbMesurementByTrain+15,3:4),1,30)]];
  endfor
  zones=[];
  for i=0:(nbTrain/nbMesurementByTrain)-1
    zones=[zones;[scanResult(i*nbMesurementByTrain+1,5)]-1];
  endfor
  tensorFlow=[tensorFlow,zones];
  idxTest=[];
  while (size(idxTest,1)<(nbTrain/nbMesurementByTrain/ratioTrainTest))
    x=randi(nbTrain/nbMesurementByTrain);
    if (sum(idxTest==x)==0)
      idxTest= [idxTest;x];
    endif
  endwhile
  testTensorFlow=[];
  for i=1:size(idxTest,1)
    testTensorFlow=[testTensorFlow;[tensorFlow(idxTest(i),:)]];
  endfor
  trainTensorFlow=[];
  for (i=1:(nbTrain/nbMesurementByTrain))
    if ((sum(idxTest==i))==0)
      trainTensorFlow=[trainTensorFlow;[tensorFlow(i,:)]];
    endif
  endfor
  printf("train matrix size:(%d,%d)--test matrix size:(%d,%d)  total:%d \n", size(trainTensorFlow,1), size(trainTensorFlow,2), size(testTensorFlow,1), size(testTensorFlow,2),size(trainTensorFlow,1)+size(testTensorFlow,1))
  csvwrite ("tensorFlow/trainTensorFlow.csv", trainTensorFlow);
  csvwrite ("tensorFlow/testTensorFlow.csv", testTensorFlow);
 # save  ("-text","tensorFlow/trainTensorFlow.csv","trainTensorFlow")
endfunction