function [avg,min,max] = testRandomRotation(rotation,count,sigmaRot)
    i=0;
%    sigmaRot=3.2; %1
    tot=0;
    min=361;
    max=-361;
while (i<count)
        i=i+1;
    rot=normrnd(rotation,sigmaRot);
    if (rot>max)
        max=rot;
    endif
    if (rot<min)
        min=rot;
    endif   
    tot=tot+rot;
endwhile
avg=tot/i;
endfunction