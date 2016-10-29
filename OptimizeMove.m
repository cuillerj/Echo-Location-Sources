function [rotation,lenToDo] = OptimizeMove(rotation,lenToDo)
%{
optimize the rotation by lowering under pi()
%}
	rotation=mod(rotation,2*pi());
	if (rotation>pi()/2 && rotation<=3*pi()/2)
		rotation=mod(pi()-rotation,pi())
		lenToDo=-lenToDo
	endif
	if (rotation>3*pi()/2)
		rotation=-mod(pi()-rotation,pi())
		lenToDo=lenToDo
	endif
return
endfunction