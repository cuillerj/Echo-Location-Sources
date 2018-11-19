function [scanResult] = ApAllocateScanZones(inFile,outFile)
printf(mfilename);
printf(ctime(time()));
#load 'extractScanResult.txt'
extractScanResult=load(inFile);
#[a,b]=size(extractScanResult);
[a,b]=size(extractScanResult);
scanResult=zeros(a,5);
for i=1:a
	zone=FindScanZones(extractScanResult(i,2),extractScanResult(i,3));
  if (!isempty(zone))
	  scanResult(i,:)=[extractScanResult(i,1),extractScanResult(i,4),extractScanResult(i,5),extractScanResult(i,6),zone];
  else
    printf(mfilename);
    printf(ctime(time()));
    printf("zone not found:(%d,%d) \n",extractScanResult(i,2),extractScanResult(i,3));
    break;
  endif
endfor
#save  ("-ascii","scanResult.txt","scanResult")
if (!isempty(zone))
  save  ("-ascii",outFile,"scanResult")
endif
endfunction