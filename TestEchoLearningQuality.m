function [robot,apRobot,TestEchoLearningMatrix,TestEchoLearningPredictions] = TestEchoLearningQuality(plotOn)
  nbPred=10;                    % define the number of predictions that will be compute for each 360° scan
  if (!exist("plotOn"))
      plotOn=false;
   endif
  %{
  TestEchoLearningMatrix.mat must be initialized ounce (/robot folder)
      TestEchoLearningMatrix=[]
      save ("-mat4-binary","TestEchoLearningMatrix.mat","TestEchoLearningMatrix")
      TestEchoLearningPredictions=[]
      save ("-mat4-binary","TestEchoLearningPredictions.mat","TestEchoLearningPredictions")
  %}
  load TestEchoLearningMatrix.mat
  [testNumber,y]=size(TestEchoLearningMatrix);
  if (testNumber==0)
    TestEchoLearningPredictions=zeros(1,4*nbPred);
    else
      load TestEchoLearningPredictions.mat

    [a,b]=size(TestEchoLearningPredictions)
    if (a!=testNumber)
        printf(mfilename);
         printf(" incoherent matrix *** ");
         printf(ctime(time()));
         return
    endif
  endif
  [apRobot,robot] = ApStartRobotRealmode();
  stopRequested=false;
  while(!stopRequested)
    testNumber=testNumber+1;
    printf(" next test number: %d *** ",testNumber)
    printf(ctime(time()));
    validate=false;
    while (validate==false && !stopRequested)
      initialLocation(1)=input("enter current location X (0 to stop): ");
      initialLocation(2)=input("enter current location Y (0 to stop): ");
      printf("do you confirm this location: X=%d Y=%d ? ",initialLocation(1),initialLocation(2));
      validate=yes_or_no("yes or no");
      if (initialLocation(1)==0 && initialLocation(2)==0 )
         printf(mfilename);
         printf(" stop requested  *** ");
         printf(ctime(time()));
         stopRequested=true;
         break
      endif
        initialLocation(3)=0;
        [available,retCode] = ApQueryCartoAvailability(apRobot,initialLocation,0,1);
        if(!available )
           printf(" current location not available: X=%d Y=%d H=%d ? ",initialLocation(1),initialLocation(2),initialLocation(3));
           printf(ctime(time()));
           validate=false;
         endif
    end
    if (!stopRequested)
        TestEchoLearningMatrix(testNumber,1)=  initialLocation(1);
        TestEchoLearningMatrix(testNumber,2)=  initialLocation(2);
        TestEchoLearningMatrix(testNumber,3)=  initialLocation(3);
        [apRobot,robot,posX,posY,posH,posProb,retCode] = ApEchoLocalizeRobotWithRotation(apRobot,robot,10,0,plotOn);
        NO=0;
        nbPasRotation=getNbStepsRotation(); % get the number of steps for a 360° rotation
        for (i=0:nbPasRotation-1)
          NO=robot.GetScanNorthOrientation(i)+NO;
        end
        NO=NO/nbPasRotation
        TestEchoLearningMatrix(testNumber,4)=  NO;
        TestEchoLearningMatrix(testNumber,5)=  retCode;
        minDistance=inf;
        for (i=1:nbPred)     
          TestEchoLearningPredictions(testNumber,(4*(i-1))+1) = posX(i);
          TestEchoLearningPredictions(testNumber,(4*(i-1))+2) = posY(i);
          TestEchoLearningPredictions(testNumber,(4*(i-1))+3) = posH(i); 
          TestEchoLearningPredictions(testNumber,(4*(i-1))+4) = posProb(i);
          distance=sqrt((posX(i)-initialLocation(1))^2+(posY(i)-initialLocation(2))^2);
          if (distance<minDistance)
            minDistance=distance;
          endif
          if (distance<15)
            printf(" found X=%d Y=%d idx=%d distance=%d \n",posX(i),posY(i),i,distance);
            TestEchoLearningMatrix(testNumber,6)=  i;  
          endif
          TestEchoLearningMatrix(testNumber,7)=minDistance;       
        end
      endif
    end
    save ("-mat4-binary","TestEchoLearningMatrix.mat","TestEchoLearningMatrix");
    save ("-mat4-binary","TestEchoLearningPredictions.mat","TestEchoLearningPredictions");
  endfunction