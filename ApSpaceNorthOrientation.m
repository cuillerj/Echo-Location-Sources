function [apRobot,robot,NorthOrientation,headingNO] = ApSpaceNorthOrientation(apRobot,robot)
% for the current robot location
% return
% NorthOrientation that is the average north orientation mesured during the scanning phase
% spaceNO that is relative NO of the robot
%
zonesXY=apGet(apRobot,"scanRefPoints");
X=apGet(apRobot,"location")(1);
Y=apGet(apRobot,"location")(2);
tmp=(zonesXY(:,1)-X).^2+(zonesXY(:,2)-Y).^2;  % look for the closest x y coordinates
[a,b]=min(tmp);                               % b get the closest index
Xa=zonesXY(b,1);                              % find corresponding X
Ya=zonesXY(b,2);							  % find corresponding YY
a=find(zonesXY(:,1)==Xa);                     % look for X inside zonesXY
b=find(zonesXY(:,2)==Ya);					  % look for Y inside zonesXY
for i=1:size(a)							      % for each X look for inside Y
	c=find(b(:,1)==a(i));                  	  % for each X look for inside Y
	if (size(c)!=0)                           
		idx=i;								  % match found get the index
	endif
endfor
%NorthOrientation=zonesXY(a(idx),:)
NorthOrientation=zonesXY(a(idx),3);            % return the closest northorientation found in zonesXY matrix
headingNO=mod(360+NorthOrientation-robot.northOrientation,360);
endfunction