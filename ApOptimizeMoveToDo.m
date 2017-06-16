function [rotation,lenToDo,forward] = ApOptimizeMoveToDo(rotationToDo,lenToDo,forward)
%{

%}

signRot=sign(rotationToDo);

rotationGrad=(rotationToDo)*pi()/180;
	if (rotationGrad>pi())
		rotationGrad=-2*pi()+rotationGrad;
		rotation=rotationGrad*180/(pi());
		return
	endif
	if (rotationGrad>pi()/2)
		rotationGrad=pi()-rotationGrad;
%		lenToDo=-lenToDo
		forward=-forward;
		rotation=rotationGrad*180/(pi());
		return
	endif
	if (rotationGrad<-pi())
		rotationGrad=2*pi()+rotationGrad;
		rotation=rotationGrad*180/(pi());
		return
	endif
	if (rotationGrad<-pi()/2)
		rotationGrad=pi()+rotationGrad;
		forward=-forward;
		rotation=rotationGrad*180/(pi());
		return
	endif
		if (rotationGrad<=pi()/2 && rotationGrad>=-pi()/2)
		rotation=rotationGrad*180/(pi());
		return
	endif
return
endfunction