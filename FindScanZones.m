function [zone] = FindScanZones(x,y)
load 'zonesXY.txt';
[c,d]=size(zonesXY);
	[r,s]=find(zonesXY(:,1:2)==[x,y]);
	t=sort(r);
	u=[t([2:size(r)]);0];
	v=t-u;
	z=find(v==0);
	zone=t(z);
endfunction