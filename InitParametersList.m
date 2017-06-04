function [parametersNameList,parametersValueList] = InitParametersList(robot);
  %{
  this function read all the parameters from the database (using Java driver) and store names and values 
  parameters will be accesible with GetParametersValueByName()
  %}
parameterNumber=robot.GetParametersNumbers()
%parametersNameList=[""];
parametersIdList=[];
parametersValueList=[];
for (i=1:parameterNumber)
	%name=robot.GetParameterName(i);
 % name=javaMethod("GetParameterName",robot,i)
 	value=robot.GetParameterNumValue(i);
%	parametersNameList=[parametersNameList;[cellstr(name)]];
 	parametersIdList=[parametersIdList;[i]];
	parametersValueList=[parametersValueList;[value]];

end

for (i=1:size(parametersIdList))
  name=javaMethod("GetParameterName",robot,i)
  printf("  value is %d \n ",parametersValueList(i))
end
endfunction