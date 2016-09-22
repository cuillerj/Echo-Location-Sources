function [robot,carto,shiftNorthXOrientation,zonesXY,all_theta] = InitOctaveRobot()
cd C:\Users\jean\Documents\Donnees\octave\robot
javaaddpath ('C:\Users\jean\Documents\Donnees\eclipse\RobotServer\bin\robot.jar');
% create and start robot
% load matrix
load carto1;
carto=carto1;
load "zonesXY.txt"
shiftNorthXOrientation=267; % shift between north and X direction in degres - that is the average mesurment
setupPath;					% define paths
robot=robotJava;            % create the java object
robot.SetTraceFileOn(1);    % route console to trace file
load all_theta;             % load the trained logistic regression matrix

endfunction