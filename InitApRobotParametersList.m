function [apRobot] = InitApRobotParametersList(apRobot,robot);
  %{
  this function read all the parameters from the database (using Java driver) and create corresponding apRobot fields
  %}
parameterNumber=robot.GetParametersNumbers()
parametersIdList=[];
parametersValueList=[];
for (i=1:parameterNumber)
 	value=robot.GetParameterNumValue(i);
 	parametersIdList=[parametersIdList;[i]];
	parametersValueList=[parametersValueList;[value]];

end

for (i=1:size(parametersIdList))
  name=javaMethod("GetParameterName",robot,i)
  printf("  value is %d \n ",parametersValueList(i))
  apRobot = setfield(apRobot,name,parametersValueList(i))     % define object fields
end
endfunction