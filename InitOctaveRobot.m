function [robot,carto,img,shiftNorthXOrientation,zonesXY,all_theta,parametersNameList,parametersValueList] = InitOctaveRobot(flat);
cd C:\Users\jean\Documents\Donnees\octave\robot
javaaddpath ('C:\Users\jean\Documents\Donnees\eclipse\RobotServer\bin\robot.jar');
% create and start robot
% load matrix
load carto1;
carto=carto1;
load carto1img;
img=carto1img;
load "zonesXY.txt"
 if (exist("robot"))
    printf("java object already exist")
 else
    printf("create java object")
    robot=robotJava;            % create the java object  
 endif
setupPath;					% define paths

robot.SetTraceFileOn(1);    % route console to trace file
%shiftNorthXOrientation=267-robot.GetParameterNumValue(1) % shift between north and X direction in degres - that is the average mesurment
if (flat==true)
	load all_thetaFlat;             % load the trained logistic regression matrix
	all_theta=all_thetaFlat;
else
	load all_theta;             % load the trained logistic regression matrix
endif
flat=true;
[parametersNameList,parametersValueList] = InitParametersList(robot,flat);
[param,value,number]=GetParametersValueByName(robot,"currentNorthOrientationReference",parametersNameList);
shiftNorthXOrientation=value
endfunction