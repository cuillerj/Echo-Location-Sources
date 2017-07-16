cd C:\Users\jean\Documents\Donnees\octave\robot
load traceMove.mat
load traceNext
load traceDet
load traceRobot
load traceEcho
traceTime=sort([traceNext(:,1);traceMove(:,1);traceRobot(:,1);traceDet(:,1);traceEcho(:,1)]);
traceDetails=zeros(size(traceTime,1),size(traceNext,2)+size(traceMove,2)+size(traceDet,2)+size(traceRobot,2)-5);
traceDetails(:,1)=traceTime;
for (i=1:size(traceNext,1))
	idx=find(traceTime==traceNext(i,1));
  for (j=1:size(idx,1))
	  traceDetails(idx(j),2:4)=traceNext(i,2:4);
  endfor
endfor
for (i=1:size(traceMove,1))
	idx=find(traceTime==traceMove(i,1));
  for (j=1:size(idx,1))
	    traceDetails(idx(j),2)=traceMove(i,2);
	    traceDetails(idx(j),5:6)=traceMove(i,3:4);
  end
endfor
for (i=1:size(traceDet,1))
	idx=find(traceTime==traceDet(i,1));
  for (j=1:size(idx,1))
	  traceDetails(idx(j),2)=traceDet(i,2);
	  traceDetails(idx(j),7:9)=traceDet(i,3:5);
  endfor
endfor
for (i=1:size(traceRobot,1))
	idx=find(traceTime==traceRobot(i,1));
  for (j=1:size(idx,1))
	    traceDetails(idx(j),2)=traceRobot(i,2);
	    traceDetails(idx(j),10:15)=traceRobot(i,3:8);
  endfor
endfor
%{
for (i=1:size(traceEcho,1))
	idx=find(traceTime==traceEcho(i,1));
  for (j=1:size(idx,1))
      traceDetails(idx(j),2)=traceEcho(i,2);
      traceDetails(idx(j),16)=traceEcho(i,3);
      traceDetails(idx(j),17)=traceEcho(i,8);
      traceDetails(idx(j),18)=traceEcho(i,4);
      traceDetails(idx(j),19)=traceEcho(i,9);
      traceDetails(idx(j),20)=traceEcho(i,5);
      traceDetails(idx(j),21)=traceEcho(i,10);
      traceDetails(idx(j),22)=traceEcho(i,6);
      traceDetails(idx(j),23)=traceEcho(i,11);
      traceDetails(idx(j),24)=traceEcho(i,7);
      traceDetails(idx(j),25)=traceEcho(i,12);
      traceDetails(idx(j),26:30)=traceEcho(i,18:22);
   endfor
endfor
%}
csvwrite("traceDetails.csv",traceDetails);
%csvwrite("traceEcho.csv",traceEcho);
%save ("-text","traceDetails.csv","traceDetails");