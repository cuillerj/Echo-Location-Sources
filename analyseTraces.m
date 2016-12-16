load traceMove
load traceNext
load traceDet
load traceRobot
load traceEcho
traceTime=sort([traceNext(:,1);traceMove(:,1);traceRobot(:,1);traceDet(:,1)]);
traceDetails=zeros(size(traceTime,1),size(traceNext,2)+size(traceMove,2)+size(traceDet,2)+size(traceRobot,2)-6);
traceDetails(:,1)=traceTime;
for (i=1:size(traceNext,1))
	idx=find(traceTime==traceNext(i,1));
	traceDetails(idx,2:4)=traceNext(i,2:4);
endfor
for (i=1:size(traceMove,1))
	idx=find(traceTime==traceMove(i,1));
	traceDetails(idx,2)=traceMove(i,2);
	traceDetails(idx,5:6)=traceMove(i,3:4);
endfor
for (i=1:size(traceDet,1))
	idx=find(traceTime==traceDet(i,1));
	traceDetails(idx,2)=traceDet(i,2);
	traceDetails(idx,7:9)=traceDet(i,3:5);
endfor
for (i=1:size(traceRobot,1))
	idx=find(traceTime==traceRobot(i,1));
	traceDetails(idx,2)=traceRobot(i,2);
	traceDetails(idx,10:15)=traceRobot(i,3:8);
endfor
csvwrite("traceDetails.csv",traceDetails);
csvwrite("traceEcho.csv",traceEcho);
%save ("-text","traceDetails.csv","traceDetails");