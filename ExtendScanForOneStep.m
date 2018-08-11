function [tensorFlowDataIn] = ExtendScanForOneStep(tensorFlowDataIn,nbMesurementByTrain)
  j=2;
  while (j<=nbMesurementByTrain)
    tensorFlowDataIn(2,j)=tensorFlowDataIn(1,j-1);
    tensorFlowDataIn(2,j+nbMesurementByTrain)=tensorFlowDataIn(1,j+nbMesurementByTrain-1);
    j=j+1;
  endwhile
  #tensorFlowDataIn(2,1)=robot.GetScanDistBack(nbMesurementByTrain-1);
  tensorFlowDataIn(2,1)=tensorFlowDataIn(1,2*nbMesurementByTrain);
  #tensorFlowDataIn(2,nbMesurementByTrain+1)=robot.GetScanDistFront(nbMesurementByTrain-1);
  tensorFlowDataIn(2,nbMesurementByTrain+1)=tensorFlowDataIn(1,nbMesurementByTrain);

  j=1;
  while (j<=nbMesurementByTrain-1)
#    tensorFlowDataIn(3,j)=robot.GetScanDistFront(j);
    tensorFlowDataIn(3,j)=tensorFlowDataIn(1,j+1);   
 #   tensorFlowDataIn(3,j+nbMesurementByTrain)=robot.GetScanDistBack(j);
    tensorFlowDataIn(3,j+nbMesurementByTrain)=tensorFlowDataIn(1,j+nbMesurementByTrain+1);;
    j=j+1;
  endwhile
 # tensorFlowDataIn(3,nbMesurementByTrain)=robot.GetScanDistBack(0);
 # tensorFlowDataIn(3,2*nbMesurementByTrain)=robot.GetScanDistFront(0);
  tensorFlowDataIn(3,nbMesurementByTrain)=tensorFlowDataIn(1,1+nbMesurementByTrain);
  tensorFlowDataIn(3,2*nbMesurementByTrain)=tensorFlowDataIn(1,1);
 endfunction