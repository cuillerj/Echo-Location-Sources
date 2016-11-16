function [parametersNameList,parametersValueList] = InitParametersList(robot);
parameterNumber=robot.GetParametersNumbers();
parametersNameList=[];
parametersValueList=[];
for (i=1:parameterNumber)
	name=robot.GetParameterName(i);
	value=robot.GetParameterNumValue(i);
	parametersNameList=[parametersNameList;[name]];
	parametersValueList=[parametersValueList;[value]];
end
endfunction