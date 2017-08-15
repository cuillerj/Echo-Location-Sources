function ApCreateAndLearnNewFlatScan()
  %{
  prepare the data before calling this procedure
      create the input file: "...robot/extractScanResult.txt" 
      use MySQL workbench "countRobotScan.sql" to check the number of scans for each idscan
        it must be 15 - if less delete the scanid if more delete the duplicated record 
       
      create a new excel file_in_loadpath
      import data with the mysql procedure extractScanRobot - select the comlete range of scans
      save as extractCountResult.txt (dos txt format)
      
 the first step will create scanResults.txt file with ApAllocateScanZones.m to fit with zonesXY.txt
 the second step will create the trainning matrix with ApcreateMatrixTrainingFlat.m
 the last step will do the learning
 
  %}
  validate=false;
  while (!validate)
    printf("did you extract the extractCountResult.txt and check the number of records are 15 a scan ?");
    validate=yes_or_no("yes or no");
  end
  more off
  printf(mfilename);
  printf(ctime(time()));
  cd C:/Users/jean/Documents/Donnees/octave/robot
  lambdamin=80;
  lambdamax=80; 
  maxIter=1000;
  nbTry=1;

  ApAllocateScanZones();
  ApCreateMatrixTrainingFlat();
  ApLearnScanRobotFlat(lambdamin,lambdamax,maxIter,nbTry)