function [param,value,number] = GetParametersValueByName(robot,input,parametersNameList)
inputSize=size(input,2);
nbParameters=size(parametersNameList,1);
param="";
value="0";
number=-1;
for (i=1:nbParameters)
	idx=strcmp(input,cellstr(parametersNameList(i,:)));
	if (idx==1)
		number=i;
		param=parametersNameList(i,:);
		value=robot.GetParameterNumValue(i);
		return
	endif
end
return
endfunction