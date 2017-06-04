function [apRobot,robot] = InitapRobot(flat);
more off
cd C:\Users\jean\Documents\Donnees\octave\robot
javaaddpath ('C:\Users\jean\Documents\Donnees\eclipse\RobotServer\bin\robot.jar');
if (!exist("flat"))  % flat or rotated IA echo location 
     flat=true;
endif
% create and start apRobot  & robot (the physical interface)
load carto1;  % load cartography
carto=carto1;
load carto1img;  % load cartography image
img=carto1img;
load "zonesXY.txt";    % load referenced scan location
scanRefPoints=zonesXY;
printf("create octave object")
apRobot=apRobot();  % create the octave robot object
 if (exist("robot"))
    printf("java object already exist")
 else
    printf("create java object")
    robot=robotJava();            % create the java robot object  
 endif
setupPath;					% define paths
%cd C:\Users\jean\Documents\Donnees\octave
%apRobotPath
robot.SetTraceFileOn(1);    % route console to trace file

if (flat==true)
	load all_thetaFlat;             % load the trained logistic regression matrix
	all_theta=all_thetaFlat;
else
	load all_theta;             % load the trained logistic regression matrix
endif
[apRobot] = InitApRobotParametersList(apRobot,robot);
%[parametersNameList,parametersValueList] = InitParametersList(robot,flat);
%[param,value,number]=GetParametersValueByName(robot,"currentNorthOrientationReference",parametersNameList);
%shiftNorthXOrientation=value;
%printf("shift between X axis and north is %d degres *** ",shiftNorthXOrientation)
%printf(ctime(time()));
apRobot = setfield(apRobot,"location",[0,0,0]);
apRobot = setfield(apRobot,"destination",[0,0,0]);
apRobot = setfield(apRobot,"locationProb",0);
apRobot = setfield(apRobot,"carto",carto);
apRobot = setfield(apRobot,"img",img);
apRobot = setfield(apRobot,"scanRefPoints",scanRefPoints);
apRobot = setfield(apRobot,"all_theta",all_theta);
%loc=apGet(apRobot,"location")
%locprob=apGet(apRobot,"locationProb")

endfunction