function [apRobot,AStarStepSmooth,forward] = ApSmoothPath(apRobot,AStarPath,AStarStep,forward,plotOn)
%{ 

 %}
 cpu1=cputime();
 nbActions=size(AStarPath,1);
 nbSteps=size(AStarStep,1);
 AStarStepSimplified=AStarStep(1,:);
 idxAction=1;
 prevAction=AStarPath(1);
 countSameAction=0;
 smoothMinLength=40;    % 
 smoothMaxLength=120;

 
 % eliminate steps that are reached with straight moves
 while (idxAction<nbActions)
	if (AStarPath(idxAction)!=prevAction)        % only keep step if there is a new kind of action
		prevAction=AStarPath(idxAction);
		AStarStepSimplified=[AStarStepSimplified;[AStarStep(idxAction,:)]];
	endif
	idxAction++;
end
	AStarStepSimplified=[AStarStepSimplified;[AStarStep(nbSteps,:)]];

%  try to smooth the moves
prevStep=AStarStepSimplified(1,:);
AStarStepSmooth=AStarStepSimplified(1,:);
i=2;

while (i<size(AStarStepSimplified-1,1))
	if (((AStarStepSimplified(i,1)-prevStep(1))^2+(AStarStepSimplified(i,2)-prevStep(2))^2)<smoothMinLength^2)
		if (((AStarStepSimplified(i+1,1)-prevStep(1))^2+(AStarStepSimplified(i+1,2)-prevStep(2))^2)<smoothMaxLength^2)
			i=i+1;
		endif
	endif
	prevStep=AStarStepSimplified(i,:);
	AStarStepSmooth=[AStarStepSmooth;[AStarStepSimplified(i,:)]];
	i=i+1;	
end
AStarStepSmooth=[AStarStepSmooth;[AStarStepSimplified(size(AStarStepSimplified,1),:)]];
AStarStepSimplified=AStarStepSmooth;
prevStep=AStarStepSimplified(1,:);
AStarStepSmooth=AStarStepSimplified(1,:);
i=2;

while (i<size(AStarStepSimplified-1,1))
	if (((AStarStepSimplified(i,1)-prevStep(1))^2+(AStarStepSimplified(i,2)-prevStep(2))^2)<smoothMinLength^2)
		if (((AStarStepSimplified(i+1,1)-prevStep(1))^2+(AStarStepSimplified(i+1,2)-prevStep(2))^2)<smoothMaxLength^2)
			i=i+1;
		endif
	endif
	prevStep=AStarStepSimplified(i,:);
	ApAStarStepSmooth=[AStarStepSmooth;[AStarStepSimplified(i,:)]];
	i=i+1;	
end

AStarStepSmooth=[AStarStepSmooth;[AStarStepSimplified(size(AStarStepSimplified,1),:)]];

% to evenutaly delete 2 identical steps
tempStep=[];
for (i=1:size(AStarStepSmooth,1))                    % to delete double steps
	if ((i==1) || (i>=2) && ((AStarStepSmooth(i,1) != AStarStepSmooth(i-1,1)) || (AStarStepSmooth(i,2) != AStarStepSmooth(i-1,2))))
		tempStep=[tempStep;[AStarStepSmooth(i,1),AStarStepSmooth(i,2)]];
	endif
end
AStarStepSmooth=tempStep;
save ("-mat4-binary","AStarStepSmooth.mat","AStarStepSmooth");
if (plotOn)
	ApAStarShowStep(apRobot,AStarStepSmooth,"Smoothed path");
	hold off
endif

endfunction