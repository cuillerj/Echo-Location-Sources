% rotate one scan result (inMatrix) for the steps number
function [outMatrix] = ApRotateMatrix(inMatrix,step)
  nbMesurementByTrain=getNbStepsRotation();
  frontMatrix=inMatrix(:,1:nbMesurementByTrain);
  backMatrix=inMatrix(:,nbMesurementByTrain+1:2*nbMesurementByTrain);
  [figureNumber] = ApPlotEchoScan(frontMatrix(1,:),backMatrix(1,:)); 
  frontMatrix=inMatrix(:,1:nbMesurementByTrain-1);
  backMatrix=inMatrix(:,nbMesurementByTrain+1:2*nbMesurementByTrain-1);
  tempMatrix=[[frontMatrix],[backMatrix]];
  for i=1:step
    tempMatrix=circshift (tempMatrix,-1,2);
 endfor
  tempFrontMatrix=tempMatrix(:,1:nbMesurementByTrain-1);
  tempBackMatrix=tempMatrix(:,nbMesurementByTrain:2*nbMesurementByTrain-2);
  frontMatrix=[tempFrontMatrix,[tempBackMatrix(:,1)]];
  backMatrix =[tempBackMatrix,[tempFrontMatrix(:,1)]];
  outMatrix=[frontMatrix,backMatrix];
  frontMatrix=outMatrix(:,1:nbMesurementByTrain);
  backMatrix=outMatrix(:,nbMesurementByTrain+1:2*nbMesurementByTrain);
  [figureNumber] = ApPlotEchoScan(frontMatrix(1,:),backMatrix(1,:)); 
endfunction