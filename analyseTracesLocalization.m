load traceMove
load traceDet
load traceEcho
traceTime=sort([traceMove(:,1);traceDet(:,1)]);
traceDetails=zeros(size(traceTime,1),size(traceMove,2)+size(traceDet,2)-2);
traceDetails(:,1)=traceTime;
for (i=1:size(traceMove,1))
	idx=find(traceTime==traceMove(i,1));
	traceDetails(idx,2)=traceMove(i,2);
	traceDetails(idx,3:5)=traceMove(i,3:5);
endfor
for (i=1:size(traceDet,1))
	idx=find(traceTime==traceDet(i,1));
	traceDetails(idx,2)=traceDet(i,2);
	traceDetails(idx,6:8)=traceDet(i,3:5);
endfor


csvwrite("traceDetailsLocalization.csv",traceDetails);

%save ("-text","traceDetails.csv","traceDetails");