function [apRobot,robot,locX,locY,locH,locCost] = ApAnalyseLastScanRotation(apRobot,robot,nbPred,rotation,plotOn)
%createMatrixAnalyseById(Id)
%javamethods(robot)
%robot.GetLastScanID()
%nbPred=5  % number of predictions to return by desc probability order
  printf(mfilename);
  printf("  ***  ");
  printf(ctime(time()))
  if (!plotOn)
        plotOn=false;
  endif
  locCost=-1;
  locCost=0; 
  locH=0;
  %load ("zonesXY.txt");
  [apRobot,robot,analyseMatRotated] = ApCreateMatrixLastScanWithRotation(apRobot,robot,rotation,plotOn);
  %analyseMatRotated=plotEchoLastScanRotation(robot,plotOn,rotation); %  create matrix analyseMat and plot data to be analysed 
  nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
  %valAngle=180/(nbPasRotation-1); % value of the angle of a step 
  %load all_thetaFlat % load matrix learnt during the traing phase
  %load analyseMatRotated.mat % load data to be analysed
  i=1;
  all_theta=apGet(apRobot,"all_theta");
  predMat = predictxVsAll(all_theta, analyseMatRotated,nbPred); % predict loaclization value zone/angle
  scanResult=apGet(apRobot,"scanResult");
  zonesXY=apGet(apRobot,"scanRefPoints");
  if (plotOn)
     load 'trainMat.mat';
    load 'trainResult.mat';
  endif
  while (i<=nbPred)
      predLoc=predMat(i,1);
      c = floor((predLoc-1)/(2*nbPasRotation-2));
      Maille=predLoc; % predicted zone 
      zonesXY(Maille,:);
      locX(i)=zonesXY(Maille,1);
      locY(i)=zonesXY(Maille,2);
      locH(i)=rotation;
      reste=predLoc-c*(2*nbPasRotation-2);
      locCost(i)=predMat(i,2);
      [x,y]=find(scanResult(:,5)==predLoc);
      scanResult(x:x+14,:);
      if (plotOn==true)
        ApPlotEchoTraining(predLoc,trainMat,trainResult); % plot training data that fit with the prediction
      endif
      i=i+1;
  end
  printf(mfilename);
  printf(" most likely locations ");
  for (i=1:nbPred)
     printf(" (%d,%d,%d) ",locX(i),locY(i),locH(i)); 
  end
  printf(ctime(time()))
