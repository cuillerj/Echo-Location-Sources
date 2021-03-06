function [AStarStepSmooth,forward] = SmoothPath(carto,AStarPath,AStarStep,forward,plotOn)
%{ 

 %}
 AStarStep
 AStarPath
 cpu1=cputime();
 nbActions=size(AStarPath,1)
 nbSteps=size(AStarStep,1)
 AStarStepSmooth=AStarStep(1,:)
 idxAction=1;
 prevAction=AStarPath(1)
 countSameAction=0;
 smoothMinLength=3;    % 
 smoothMaxLength=4;
 while (idxAction<nbActions)
	if (AStarPath(idxAction)==prevAction)
		countSameAction++
		newAction=false;
	else
		newAction=true;
		prevAction=AStarPath(idxAction)
	endif
	if (newAction==true && countSameAction >=smoothMinLength)
%		for i = idxAction-countSameAction+1:idxAction
			AStarStepSmooth=[AStarStepSmooth;[AStarStep(idxAction,:)]]
%		endfor
	countSameAction=0;
	else
		i=smoothMaxLength;
		while(i>1)
			idxStart=idxAction-countSameAction+1
			idxEnd=min(nbSteps,idxAction-countSameAction+i+1)
%			heading=atan((AStarStep(idxEnd,1)-AStarStep(idxStart,1)/(AStarStep(idxEnd,1)-AStarStep(idxStart,1))));
				[rotation,distance,possible]=CheckStraightMovePossibility(carto,AStarStep(idxStart,1),AStarStep(idxStart,2),0,AStarStep(idxEnd,1),AStarStep(idxEnd,2));
				if (possible==1)
					AStarStepSmooth=[AStarStepSmooth;[AStarStep(idxEnd,:)]]
					idxAction=idxEnd-1;
					i=0;
				endif
				if (idxEnd>=nbSteps)
%					AStarStepSmooth=[AStarStepSmooth;[AStarStep(nbSteps,:)]];
					i=0;
				endif
			i--;
		end
	endif
	idxAction++;
end
AStarStepSmooth=[AStarStepSmooth;[AStarStep(nbSteps,:)]];
tempStep=[];
for (i=1:size(AStarStepSmooth,1))                    % to delete double steps
	if ((i==1) || (i>=2) && ((AStarStepSmooth(i,1) != AStarStepSmooth(i-1,1)) || (AStarStepSmooth(i,2) != AStarStepSmooth(i-1,2))))
		tempStep=[tempStep;[AStarStepSmooth(i,1),AStarStepSmooth(i,2)]];
	endif
end
AStarStepSmooth=tempStep;
save ("-mat4-binary","AStarStepSmooth.mat","AStarStepSmooth");
if (plotOn)
	AStarShowStep(AStarStepSmooth,"Smoothed path");
	hold off
endif

endfunction