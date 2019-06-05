function [] = ToolCreateSRTfile(fileName,deltaSec)
    % this tool creat a subtittle file for the video record of the robot according to traceMoce and traceRobot matrix
    if (!exist("flat"))  % flat or rotated IA echo location 
     flat=0;
    endif
    load("traceMove");
    load("traceRobot");
    retCodeIdx=find(traceRobot(:,8)!=0);
    nbLine=size(traceMove,1);
    idx=1;
    step=1;
    while (nbLine-idx>=1)
        if (idx==1)
            csvwrite (fileName, idx);
             beginTime=traceMove(idx,1)+deltaSec;
        else
            csvwrite (fileName, step, "-append");
        endif
        for (i=1:size(retCodeIdx))
            if(idx>1 && traceRobot(retCodeIdx(i),1)>traceMove(idx-1,1) && traceRobot(retCodeIdx(i),1)<traceMove(idx,1))
                 startTime=max(traceRobot(retCodeIdx(i),1)-beginTime,0);
                 stopTime=startTime+5;
                 [hoursStart, minsStart, secsStart] = ToolSec2hms(startTime);
                 [hoursStop, minsStop secsStop] = ToolSec2hms(stopTime);
                 lineOut=strcat(mat2str(hoursStart),":",mat2str(minsStart),":",mat2str(round(secsStart)),",000 -->"," ",mat2str(hoursStop),":",mat2str(minsStop),":",mat2str(round(secsStop)));
                 csvwrite (fileName, lineOut,"delimiter", "", "-append");
                lineOut=strcat("loop-",mat2str(traceMove(idx,2))," robot return code:",mat2str(traceRobot(retCodeIdx(i),8)));
                csvwrite (fileName, lineOut,"delimiter", "", "-append"); 
                csvwrite (fileName, "\n", "-append"); 
                step++; 
                csvwrite (fileName, step, "-append");        
            endif
        endfor 
         startTime=max(traceMove(idx,1)-beginTime,0);
         stopTime=max(min(traceMove(idx+1,1)-beginTime,traceMove(idx,1)-beginTime+10),0);
         [hoursStart, minsStart, secsStart] = ToolSec2hms(startTime);
         [hoursStop, minsStop secsStop] = ToolSec2hms(stopTime);
         lineOut=strcat(mat2str(hoursStart),":",mat2str(minsStart),":",mat2str(round(secsStart)),",000 -->"," ",mat2str(hoursStop),":",mat2str(minsStop),":",mat2str(round(secsStop)));
         csvwrite (fileName, lineOut,"delimiter", "", "-append");
         lineOut=strcat("loop-",mat2str(traceMove(idx,2))," move:",mat2str(traceMove(idx,3)),"deg <>",mat2str(traceMove(idx,4)),"cm");
         csvwrite (fileName, lineOut,"delimiter", "", "-append");
         csvwrite (fileName, "\n", "-append");
         idx++;
         step++;
     endwhile
            for (i=1:size(retCodeIdx))
            if(idx>1 && traceRobot(retCodeIdx(i),1)>traceMove(idx-1,1) && traceRobot(retCodeIdx(i),1)<traceMove(idx,1))
                 startTime=max(traceRobot(retCodeIdx(i),1)-beginTime,0);
                 stopTime=startTime+5;
                 [hoursStart, minsStart, secsStart] = ToolSec2hms(startTime);
                 [hoursStop, minsStop secsStop] = ToolSec2hms(stopTime);
                 csvwrite (fileName, step, "-append");      
                 lineOut=strcat(mat2str(hoursStart),":",mat2str(minsStart),":",mat2str(round(secsStart)),",000 -->"," ",mat2str(hoursStop),":",mat2str(minsStop),":",mat2str(round(secsStop)));
                csvwrite (fileName, lineOut,"delimiter", "", "-append");
                lineOut=strcat("loop-",mat2str(traceMove(idx,2))," robot return code:",mat2str(traceRobot(retCodeIdx(i),8)));
                csvwrite (fileName, lineOut,"delimiter", "", "-append"); 
                csvwrite (fileName, "\n", "-append"); 
                step++; 
            endif
        endfor 
         csvwrite (fileName, step, "-append");
         startTime=max(traceMove(idx,1)-beginTime,0);
         stopTime=startTime+7;
         [hoursStart, minsStart, secsStart] = ToolSec2hms(startTime);
         [hoursStop, minsStop secsStop] = ToolSec2hms(stopTime);
         lineOut=strcat(mat2str(hoursStart),":",mat2str(minsStart),":",mat2str(round(secsStart)),",000 -->"," ",mat2str(hoursStop),":",mat2str(minsStop),":",mat2str(round(secsStop)));
         csvwrite (fileName, lineOut,"delimiter", "", "-append");
         lineOut=strcat("loop-",mat2str(traceMove(idx,2))," move:",mat2str(traceMove(idx,3)),"deg <>",mat2str(traceMove(idx,4)),"cm");
         csvwrite (fileName, lineOut,"delimiter", "", "-append");
         csvwrite (fileName, "\n", "-append");
        
    endfunction