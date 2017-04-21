function [] = AllocateScanZones()
load 'extractScanResult.txt'
[a,b]=size(extractScanResult);
scanResult=zeros(a,5);
for i=1:a
	zone=FindScanZones(extractScanResult(i,2),extractScanResult(i,3));
	scanResult(i,:)=[extractScanResult(i,1),extractScanResult(i,4),extractScanResult(i,5),extractScanResult(i,6),zone];
endfor
save  ("-ascii","scanResult.txt","scanResult")
endfunction