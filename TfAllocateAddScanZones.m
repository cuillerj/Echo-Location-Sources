#{
load new raw scan data, replace (X,Y) by the corresponding zone number and save a new file scanAddResult
in order to feed tensorflow learing
#}
function [] = TfAllocateAddScanZones()
load 'extractAddScan.txt'
[a,b]=size(extractAddScan);
scanResult=zeros(a,5);
listZone=[]
prevZone=[-1,-1];
for i=1:a
  if ([extractAddScan(i,2),extractAddScan(i,3)] != prevZone)
    prevZone=[extractAddScan(i,2),extractAddScan(i,3)]
    listZone=[listZone;[prevZone]]
	  zone=FindScanZones(extractAddScan(i,2),extractAddScan(i,3));
	  scanAddResult(i,:)=[extractAddScan(i,1),extractAddScan(i,4),extractAddScan(i,5),extractAddScan(i,6),zone];
   endif
endfor
csvwrite ("tensorFlow/scanAddResult.csv", scanAddResult, "-append");
endfunction