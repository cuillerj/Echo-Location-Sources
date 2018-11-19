#{
load new raw scan data, replace (X,Y) by the corresponding genrated zone number and save a new file scanFlatResult.csv
create flatZonesList.csv that contains zones <-> (x,y) mapping
scanFlatResult.csv will be input for tensorflow learning (flat and rotated)
1) use extractNewEchoScan.sql to extract data export fname.csv
2) open fname.csv with excel
  convert rows  
  save as fname.txt
3) TfAddNewScan(fname,true) or TfAddNewScan(fname,false)
    true to refresh previous data
    false to add data to previous one

at the end call 
    TfPartiallyExtendScanResult();
    TfCreatePartiallyFlatMatrix()
    
4) check manualy files creation C:\Users\jean\Documents\Donnees\octave\robot\tensorFlow\
     testTensorFlowZeroCentered.csv
     TfPaamters.csv
     trainTensorFlowZeroCentered.csv 
     validationTensorFlowZeroCentered.csv    
     zonesList.csv
     testFeatruesData.csv
     trainFeaturesData.csv
     trainAndTestFeaturesData.csv
     
5) ready to start tensor flow learning
#}
function [] = TfAddNewScan(fname,raz)
  printf(mfilename);
  printf(" start ")
  printf(ctime(time()))
  sensorDistanceLimit=getSensorDistanceLimit();  # will replace 0 value mesurment
  if (!exist("fname"))  % real is default mode 
      fname='newScan.txt';
  endif
  if (!exist("raz"))  % false is default mode 
      raz=false;
  endif
  newScan=load(fname);
  [a,b]=size(newScan);
  if (floor(a/getNbStepsRotation())==a/getNbStepsRotation())  
    if (raz)
      flatData=[];
    else
      load('learningScanFlatData.mat');
    endif
    for (i=1:a)
      flatData=[flatData;[newScan(i,1),newScan(i,2),newScan(i,3),newScan(i,4),newScan(i,5),newScan(i,6)]];
    endfor
    flatData(:,5)=flatData(:,5)+(flatData(:,5)==0)*sensorDistanceLimit;
    flatData(:,6)=flatData(:,6)+(flatData(:,6)==0)*sensorDistanceLimit;
    save("-mat4-binary","learningScanFlatData.mat",'flatData');
    scanResult=zeros(a,5);
    zone=-1;
    flatZonesList=[zone,[0,0]];
    for i=1:size(flatData,1)
      checkIn=flatZonesList(:,2:3)==[flatData(i,2),flatData(i,3)];
       [s1,s2]=max(sum(checkIn,2));
       if (s1==2) #found
        idxZone=flatZonesList(s2,1);
      else
        zone=zone+1;
        idxZone=zone;
        flatZonesList=[flatZonesList;[zone,[flatData(i,2),flatData(i,3)]]];        
      endif

      scanAddResult(i,:)=[flatData(i,1),flatData(i,4),flatData(i,5),flatData(i,6),idxZone];
    endfor
    flatZonesList=flatZonesList(2:(size(flatZonesList,1)),:);
    csvwrite ("scanFlatResult.csv", scanAddResult);
    csvwrite ("flatZonesList.csv", flatZonesList);
  else
    printf(mfilename);
    printf("records number (%d) must be a multiple of step size (%d)",a,getNbStepsRotation())
    printf(ctime(time()))
 endif
 TfPartiallyExtendScanResult();
 TfCreatePartiallyFlatMatrix();
endfunction