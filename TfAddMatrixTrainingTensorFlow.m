
function [addTensorFlow] = TfAddMatrixTrainingTensorFlow()
  nbMesurementByTrain=getNbStepsRotation(); % get the number of steps for a 180 rotation
  load ('scanAddResult.txt');
  nbTrain=size(scanAddResult,1);
  printf("training records number:%d,number of mesurment by train:%d scan nuùber:%d \n",nbTrain,nbMesurementByTrain,nbTrain/nbMesurementByTrain)
  addTensorFlow=[];
  for i=0:nbTrain/nbMesurementByTrain-1
    #scanResult(i*nbMesurementByTrain+1:i*nbMesurementByTrain+15,3:4);
    addTensorFlow=[addTensorFlow;[reshape(scanAddResult(i*nbMesurementByTrain+1:i*nbMesurementByTrain+15,3:4),1,30)]];
  endfor
  zones=[];
  for i=0:(nbTrain/nbMesurementByTrain)-1
    zones=[zones;[scanAddResult(i*nbMesurementByTrain+1,5)]-1];
  endfor
  addTensorFlow=[addTensorFlow,zones];
  printf("add matrix size:(%d,%d)-- \n", size(addTensorFlow,1), size(addTensorFlow,2))
  csvwrite ("tensorFlow/trainTensorFlow.csv", addTensorFlow,"-append");
endfunction