function [apRobot,robot] = ApInitApRobot(flat);
more off
printf(mfilename);
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
load "optimalPath.txt";    % load optimal path for routing
scanRefPoints=zonesXY;
printf(mfilename);
if (exist("apRobot"))
    printf(" octave object already exist *** \n")
 else
    printf(" create octave object *** \n")
    apRobot=apRobot();  % create the octave robot object
 endif
 printf(mfilename);
 if (exist("robot"))
    printf(" java object already exist *** \n")
 else
    printf(" create java object *** \n")
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
apRobot = setfield(apRobot,"gyroLocation",[0,0,0]);
apRobot = setfield(apRobot,"destination",[0,0,0]);
apRobot = setfield(apRobot,"newTarget",true);
apRobot = setfield(apRobot,"locationProb",0);
apRobot = setfield(apRobot,"callFrom",10);   % to identify the main octave function
apRobot = setfield(apRobot,"typeWait",0);   % to identify the main octave function
apRobot = setfield(apRobot,"waitFor",0);   % to identify the main octave function
apRobot = setfield(apRobot,"carto",carto);
apRobot = setfield(apRobot,"img",img);
apRobot = setfield(apRobot,"scanRefPoints",scanRefPoints);
apRobot = setfield(apRobot,"optimalPath",optimalPath);
apRobot = setfield(apRobot,"all_theta",all_theta);
apRobot = setfield(apRobot,"saveLocation",apGet(apRobot,"location"));
apRobot = setfield(apRobot,"hardLocation",apGet(apRobot,"location"));  % location based on robot compyuting
apRobot = setfield(apRobot,"gyroLocation",apGet(apRobot,"location"));   % location based on gyroscope
apRobot = setfield(apRobot,"subsytemLeft",apGet(apRobot,"location")); % location provided by BNO subsystem left wheel
apRobot = setfield(apRobot,"subsytemRight",apGet(apRobot,"location")); % location provided by BNO subsystem right wheel
apRobot = setfield(apRobot,"nextLocation",apGet(apRobot,"location"));
RobotWidth=apGet(apRobot,"iRobotWidth");
WheelDiameter=apGet(apRobot,"iLeftWheelDiameter");
WheelEncoderHoles=apGet(apRobot,"leftWheelEncoderHoles");
apRobot = setfield(apRobot,"angleOneHole",((pi*WheelDiameter)/WheelEncoderHoles)/(pi*RobotWidth)*360);
apRobot = setfield(apRobot,"distanceOneHole",((pi*WheelDiameter)/WheelEncoderHoles));
%loc=apGet(apRobot,"location")
%locprob=apGet(apRobot,"locationProb")

endfunction