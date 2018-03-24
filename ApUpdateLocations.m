 function [apRobot,robot]=ApUpdateLocations(apRobot,robot)
         shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter")/10;
         saveLocation=apGet(apRobot,"saveLocation");
         rotationToDo=apGet(apRobot,"lastRotation");
         lenToDo=apGet(apRobot,"lastMove");
         nextLoc(1)=round(saveLocation(1)+cos((saveLocation(3)+rotationToDo)*pi()/180)*(lenToDo+shiftEchoVsRotationCenter)-shiftEchoVsRotationCenter*cos(saveLocation(3)*pi()/180));
         nextLoc(2)=round(saveLocation(2)+sin((saveLocation(3)+rotationToDo)*pi()/180)*(lenToDo+shiftEchoVsRotationCenter)-shiftEchoVsRotationCenter*sin(saveLocation(3)*pi()/180));
         nextLoc(3)=mod(saveLocation(3)+rotationToDo,360);
         apRobot = setfield(apRobot,"nextLocation",nextLoc);         
 %         apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);
%          [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot);       % wait for updated information from robot  
                      % get and store BNO subsystem data          
          apRobot = setfield(apRobot,"subsytemLeft",[robot.BNOLeftPosX,robot.BNOLeftPosY,robot.BNOLocHeading]);
          apRobot = setfield(apRobot,"subsytemRight",[robot.BNORightPosX,robot.BNORightPosY,robot.BNOLocHeading]);  
              % compute locations   
 %             apGet(apRobot,"gyroLocation")
  %            apGet(apRobot,"lastRotation")
  %        gyroBasedH=mod(apGet(apRobot,"gyroLocation")(3)+apGet(apRobot,"lastRotation"),360);
          gyroBasedH=mod(robot.BNOLocHeading,360);
          prevGyroX= apGet(apRobot,"gyroLocation")(1);
          prevGyroY= apGet(apRobot,"gyroLocation")(2);          
          gyroBasedX=round(prevGyroX+(apGet(apRobot,"lastMove")+shiftEchoVsRotationCenter)*cos(gyroBasedH*pi()/180)-shiftEchoVsRotationCenter*cos(apGet(apRobot,"saveGyroLocation")(3)*pi()/180));
          gyroBasedY=round(prevGyroY+(apGet(apRobot,"lastMove")+shiftEchoVsRotationCenter)*sin(gyroBasedH*pi()/180)-shiftEchoVsRotationCenter*sin(apGet(apRobot,"saveGyroLocation")(3)*pi()/180));
%          gyroBasedX=gyroBasedX-shiftEchoVsRotationCenter*cos(gyroBasedH*pi()/180); % set position to rotation center
%          gyroBasedY=gyroBasedY-shiftEchoVsRotationCenter*sin(gyroBasedH*pi()/180); % set position to rotation center
          gyroBasedLoc=[gyroBasedX,gyroBasedY,gyroBasedH];
          apRobot = setfield(apRobot,"gyroLocation",gyroBasedLoc);
          robot.GetHardPosX();
          robot.GetHardPosY();
          robot.GetHeading();
          apRobot = setfield(apRobot,"hardLocation",[robot.GetHardPosX(),robot.GetHardPosY(),robot.GetHeading()]);
          % prepare data for localization determination
          retry=0;
          robot.ValidHardPosition(); 
          while (robot.BNOLocFlag!=0 && retry<=3)
             pause(1);
             retry=retry+1;
             robot.ValidHardPosition(); % the new hard position will be taken into account by java code
          end
               
endfunction   