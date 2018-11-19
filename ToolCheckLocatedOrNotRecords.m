function [rowToCheckList,fileIn] = ToolCheckLocatedOrNotRecords()
    printf(mfilename);
    printf(" *** ")
    printf(ctime(time()))
   fileIn=csvread("determinationLocatedOrNot.txt");
   [r,c]=size(fileIn);
   lastRecords=false;
   stepIdx=1;
   loopIdIdx=2;
   countDetIdx=3;
   detXIdx=4;
   detYIdx=5;
   detHIdx=6;
   probCumIdx=7;
   TheoPosXIdx=10;
   TheoPosYIdx=11;
   deltaDistIdx=12;
   TfposXIdx=13;
   TfposYIdx=14;
   TfposProb=15;
   locatedIdx=22;
   consistantIdx=23;
   checkedIdx=24;
   realModeIdx=25;
   idx=r;
    i=1;
   while (!lastRecords && idx>0)
     if (fileIn(idx,realModeIdx) && fileIn(idx,stepIdx)==1 && !fileIn(idx,checkedIdx))
       rowToCheckList(i,1)=idx;
       if (i==1)
           rowToCheckList(i,2)=r;
       else
           rowToCheckList(i,2)=rowToCheckList(i-1,1)-1;
       endif
       i++;
     endif
     idx--;
   endwhile
   rowToCheckList=flipud(rowToCheckList);
  for (i=1:size(rowToCheckList,1))
    count=0;
    for (j=rowToCheckList(i,1):rowToCheckList(i,2))
       fileIn(j,:);
       if (fileIn(j,stepIdx)==1)
        printf("\n")
       endif
       printf("row:%d step:%d loop:%d countDet:%d ** detLoc:(%d,%d,%d) detProb:%.1f%% ** theoLoc:(%d,%d) theoDelta:%dcm ** TfLoc:(%d,%d) TfPorb:%.1f%% ** (Located,consistant):(%d,%d) \n",j,
         fileIn(j,stepIdx),fileIn(j,loopIdIdx),fileIn(j,countDetIdx),
         fileIn(j,detXIdx),fileIn(j,detYIdx),fileIn(j,detHIdx),fileIn(j,probCumIdx)*100,
         fileIn(j,TheoPosXIdx),fileIn(j,TheoPosYIdx),fileIn(j,deltaDistIdx),
         fileIn(j,TfposXIdx),fileIn(j,TfposYIdx),fileIn(j,TfposProb)*100,
         fileIn(j,locatedIdx),fileIn(j,consistantIdx))
       i++;
       count++;
     endfor
  endfor
  fflush(stdout);
  input("enter checking phase");
  for (i=1:size(rowToCheckList,1))
    count=0;
    for (j=rowToCheckList(i,1):rowToCheckList(i,2))
      printf("row: %d do you check it ? \n" ,j);
      fflush(stdout);
      checked=yes_or_no(" do you check it ? " );
      if (checked)
        fileIn(j,checkedIdx)=true;
        printf("(located,consistant) (%d,%d) do you validate it ?\n", fileIn(j,locatedIdx),fileIn(j,consistantIdx))
        fflush(stdout);
        validate=yes_or_no("yes or no");
        if (!validate)
            printf(" located :%d do you validate it ?\n", fileIn(j,locatedIdx))
           fflush(stdout);
            validate=yes_or_no("yes or no");
            if (!validate)
                fileIn(j,locatedIdx)=!fileIn(j,locatedIdx);
                printf("consistant %d do you validate it ? \n", fileIn(j,consistantIdx))
                fflush(stdout);
                validate=yes_or_no("yes or no");
                if (!validate)
                  fileIn(j,consistantIdx)=!fileIn(j,consistantIdx); 
                endif              
            endif
        endif
      endif
    endfor
  endfor
  dlmwrite("determinationLocatedOrNot.txt",fileIn,'delimiter',',');
endfunction