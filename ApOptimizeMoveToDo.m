function [rotation,lenToDo,forward] = ApOptimizeMoveToDo(rotationToDo,lenToDo,forward,shiftEchoVsRotationCenter)
%{

%}
signRot=sign(rotationToDo);
rotation=rotationToDo;
%rotationToDo=mod(rotationToDo,360);
rotationGrad=(rotationToDo)*pi()/180;
 if ((abs(rotationGrad)<=pi()/2 && signRot>0))
    rotation=rotationToDo;
	%	rotation=round(rotationGrad*180/(pi()))*signRot;
		return
 elseif ((abs(rotationGrad)<=pi()/2 && signRot<0))
     rotation=rotationToDo;
    return

 elseif ((abs(rotationGrad)>pi()/2 && signRot>0))
    rotationGrad=rotationGrad-pi();
    rotation=round(rotationGrad*180/(pi()));
 %   forward=-forward;
    lenToDo=-round(lenToDo+2*shiftEchoVsRotationCenter);
    return
  elseif ((abs(rotationGrad)>pi()/2 && abs(rotationGrad)<=pi() && signRot<0))
    rotationGrad=rotationGrad+pi();
    rotation=round(rotationGrad*180/(pi()));
 %   forward=-forward;
    lenToDo=-round(lenToDo+2*shiftEchoVsRotationCenter);
    return
  elseif ((abs(rotationGrad)>pi()&& signRot<0))
    rotation=mod(rotation,360);
 %   forward=-forward;
    lenToDo=round(lenToDo);
    return
	endif
  %{
	if (rotationGrad>3*pi()/2)
		rotationGrad=-2*pi()+rotationGrad;
		rotation=rotationGrad*180/(pi());
		return
	endif
  	if (rotationGrad >= 180 && rotationGrad<3*pi()/2)
		rotationGrad=-rotationGrad;
    forward=-forward;
    lenToDo=lenToDo+2*shiftEchoVsRotationCenter;     
		rotation=rotationGrad*180/(pi());
		return
	endif
	if (rotationGrad>pi()/2)
		rotationGrad=rotationGrad-pi();
%		lenToDo=-lenToDo
		forward=-forward;
    lenToDo=lenToDo+2*shiftEchoVsRotationCenter;         %  adjust lenght for backward move
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
    lenToDo=lenToDo+2*shiftEchoVsRotationCenter;          %  adjust lenght for backward move
		rotation=rotationGrad*180/(pi());
		return
	endif
		if (rotationGrad<=pi()/2 && rotationGrad>=-pi()/2)
		rotation=rotationGrad*180/(pi());
		return
	endif
  if (lenToDo>=apGet(apRobot,"minDistToBeDone")/2)  
    lenToDo=max(lenToDo,apGet(apRobot,"minDistToBeDone"));
  endif
  if (rotation>=apGet(apRobot,"minRotToBeDone")/2)  
    rotation=max(rotationToDo,apGet(apRobot,"minRotToBeDone"));
  endif
  if (rotation<apGet(apRobot,"minRotToBeDone")/2 && lenToDo<apGet(apRobot,"minDistToBeDone")/2)
     rotation=max(rotationToDo,apGet(apRobot,"minRotToBeDone"));   
     lenToDo=0;
  endif
  %}

return
endfunction