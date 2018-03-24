% create a matrix for one scan rotated by "rotation" degres
function [apRobot,robot,analyseMatRotated] = ApCreateMatrixLastScanWithRotation(apRobot,robot,rotation,plotOn)
  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()))
  nbMesurementByTrain=getNbStepsRotation();
  %load ('scanToAnalyse.txt');
  scanToAnalyse=zeros(nbMesurementByTrain,3);
  j=1;
  while (j<=nbMesurementByTrain)
    scanToAnalyse(j,1)=robot.GetScanAngle(j-1);
    scanToAnalyse(j,2)=robot.GetScanDistFront(j-1);
    scanToAnalyse(j,3)=robot.GetScanDistBack(j-1);
    j=j+1;
  end				
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
  if (plotOn)
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