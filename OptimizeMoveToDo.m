function [rotation,lenToDo,forward] = OptimizeMoveToDo(rotationToDo,lenToDo,forward)
%{

%}
rotationToDo
forward
signRot=sign(rotationToDo)
%rotationGrad=(mod(((rotationToDo)*pi()/180)+2*pi(),2*pi()))*signRot
rotationGrad=(rotationToDo)*pi()/180
	if (rotationGrad>pi())
		printf("*1 ")
		rotationGrad=-2*pi()+rotationGrad
		rotation=rotationGrad*180/(pi())
		return
	endif
	if (rotationGrad>pi()/2)
	printf("*2 ")
		rotationGrad=pi()-rotationGrad
%		lenToDo=-lenToDo
		forward=-forward;
		rotation=rotationGrad*180/(pi())
		return
	endif
	if (rotationGrad<-pi())
		printf("*3 ")
		rotationGrad=2*pi()+rotationGrad
		rotation=rotationGrad*180/(pi())
		return
	endif
	if (rotationGrad<-pi()/2)
		printf("*4 ")
		rotationGrad=pi()+rotationGrad
%		lenToDo=-lenToDo
		forward=-forward;
		rotation=rotationGrad*180/(pi())
		return
	endif
		if (rotationGrad<=pi()/2 && rotationGrad>=-pi()/2)
		printf("*5 ")
		rotation=rotationGrad*180/(pi())
		return
	endif
%	rotation=rotationGrad*180/(pi())
return
endfunction