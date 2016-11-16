function [robot,carto,img,shiftNorthXOrientation,zonesXY,all_theta,parametersNameList,parametersValueList] = InitOctaveRobot();
cd C:\Users\jean\Documents\Donnees\octave\robot
javaaddpath ('C:\Users\jean\Documents\Donnees\eclipse\RobotServer\bin\robot.jar');
% create and start robot
% load matrix
load carto1;
carto=carto1;
load carto1img;
img=carto1img;
load "zonesXY.txt"
setupPath;					% define paths
robot=robotJava;            % create the java object
robot.SetTraceFileOn(1);    % route console to trace file
%shiftNorthXOrientation=267-robot.GetParameterNumValue(1) % shift between north and X direction in degres - that is the average mesurment

load all_theta;             % load the trained logistic regression matrix
[parametersNameList,parametersValueList] = InitParametersList(robot);
[param,value,number]=GetParametersValueByName(robot,"currentNorthOrientationReference",parametersNameList);
shiftNorthXOrientation=267-value
endfunction