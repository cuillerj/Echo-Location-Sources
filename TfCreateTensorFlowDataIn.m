% create a matrix for one scan rotated by "rotation" degres
function [apRobot,robot,tensorFlowDataIn,extrapolatedTensorFlowDataIn] = TfCreateTensorFlowDataIn(apRobot,robot,rotation,plotOn)
  if (!exist("plotOn"))
    plotOn=false;
  endif
  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()))
  nbMesurementByTrain=getNbStepsRotation();
  sensorDistanceLimit=getSensorDistanceLimit();  # will replace 0 value mesurment
  incrementValue=get360StepSizeDegre();  # nb of ° incrementation by step 
  stepSize=round(180/(nbMesurementByTrain-1));
  %load ('scanToAnalyse.txt');
  scanToAnalyse=zeros(nbMesurementByTrain,3);
  j=1;
  tensorFlowDataIn=zeros(3,2*nbMesurementByTrain);
  while (j<=nbMesurementByTrain)
    tensorFlowDataIn(1,j)=robot.GetScanDistFront(j-1);
    tensorFlowDataIn(1,j+nbMesurementByTrain)=robot.GetScanDistBack(j-1);
   j=j+1;
  endwhile
  
  j=2;
  while (j<=nbMesurementByTrain)
    tensorFlowDataIn(2,j)=tensorFlowDataIn(1,j-1);
    tensorFlowDataIn(2,j+nbMesurementByTrain)=tensorFlowDataIn(1,j+nbMesurementByTrain-1);
    j=j+1;
  endwhile
  #{
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
  #}
  #newScan=tensorFlowDataIn(1,:);
  [tensorFlowDataIn] = ExtendScanForOneStep(tensorFlowDataIn,nbMesurementByTrain);
  tensorFlowDataIn(:,:)=tensorFlowDataIn(:,:)+(tensorFlowDataIn(:,:)==0)*sensorDistanceLimit;
  [dl,dc]=size(tensorFlowDataIn);
  avgDist=sensorDistanceLimit()/2;
  tensorFlowDataIn=tensorFlowDataIn-avgDist;   # to center mesurments to 0
  csvwrite ("tensorFlow/tensorFlowDataIn.csv", tensorFlowDataIn);
    # add extraplotated data
  extrapolatedTensorFlowDataIn = ExtrapolateScan(tensorFlowDataIn,incrementValue);
  csvwrite ("rotatedTensorFlow/tensorFlow360DataIn.csv", extrapolatedTensorFlowDataIn);

    j=1;
    while (j<=nbMesurementByTrain)
      scanToAnalyse(j,1)=robot.GetScanAngle(j-1);
      scanToAnalyse(j,2)=robot.GetScanDistFront(j-1);
      scanToAnalyse(j,3)=robot.GetScanDistBack(j-1);
      j=j+1;
    end
  csvwrite ("tensorFlow/newRequest.txt",time)
  csvwrite ("rotatedTensorFlow/newRequest.txt",time)
  if (plotOn) 
    nbRec=size(scanToAnalyse,1);
    analyseMat=zeros(1,362);
    j=1;
    i=1;
    pixelBF=zeros(2,181);
    while (j<=nbRec)
            angle=scanToAnalyse(j,1);
            k=angle;
            pixelBF(1,k+1)=scanToAnalyse(j,2);
            pixelBF(2,k+1)=scanToAnalyse(j,3);
            if (j < nbRec)
              angleNext=scanToAnalyse(j+1,1);
              interval=1;
                while (k < angleNext )
                  pixelBF(1,k+2)=scanToAnalyse(j,2)+(scanToAnalyse(j+1,2)-scanToAnalyse(j,2))/(angleNext-angle)*interval; 
                  pixelBF(2,k+2)=scanToAnalyse(j,3)+(scanToAnalyse(j+1,3)-scanToAnalyse(j,3))/(angleNext-angle)*interval;
                  if (scanToAnalyse(j,2)==0 || scanToAnalyse(j+1,2)==0)  
                    pixelBF(1,k+2)=0;
                  end
                  if (scanToAnalyse(j,3)==0 || scanToAnalyse(j+1,3)==0)
                    pixelBF(2,k+2)=0;
                  end
                  k=k+1;
                  interval=interval+1;
                end
            end
            j=j+1;
        analyseMat(1,:)=reshape (pixelBF,1,[]);
    end
    analyseMatRotated(1,:)=reshape (pixelBF,1,[]);
    %size(analyseMat)
    %save  ("-mat4-binary","analyseMat.mat","analyseMat")

          pixelBFTr=pixelBF';
        for i=(1:rotation)
          pixelBFTr=circshift(pixelBFTr,1);
          x=pixelBFTr(1,1);
          y=pixelBFTr(1,2);
          pixelBFTr(1,1)=y;
          pixelBFTr(1,2)=x;
        endfor
          pixelBF=pixelBFTr';

         angle=linspace(1,181,181); % 
         if plotOn==true
          col=['r','g','b','m','c']; % graph color 
          figure();
          title (0);
          hold on;
        endif
         Z=reshape(analyseMatRotated(1,:),2,181);  % reshape matrix (1 362) > (2 181)
          dX=Z(1,:);
          dY=Z(2,:);
          angleR=angle*pi()/180;
          X=dX.*cos(angleR);
          Y=dX.*sin(angleR);
        if plotOn==true
          plot (X,Y,col(3));
        endif
          X=-dY.*cos(angleR);
          Y=-dY.*sin(angleR);
        if plotOn==true
          plot (X,Y,col(3));
          hold off;
        endif
   endif
 endfunction
%save  ("-mat4-binary","analyseMatRotated.mat","analyseMatRotated")