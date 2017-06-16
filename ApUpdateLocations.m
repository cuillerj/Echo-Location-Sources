 function [apRobot,robot]=ApUpdateLocations(apRobot,robot)
         shiftEchoVsRotationCenter=apGet(apRobot,"shiftEchoVsRotationCenter");
         saveLocation=apGet(apRobot,"saveLocation");
         rotationToDo=apGet(apRobot,"lastRotation");
         lenToDo=apGet(apRobot,"lastMove");
         nextLoc(1)=round(saveLocation(1)+cos((saveLocation(3)+rotationToDo)*pi()/180)*lenToDo);
         nextLoc(2)=round(saveLocation(2)+sin((saveLocation(3)+rotationToDo)*pi()/180)*lenToDo);
         nextLoc(3)=mod(saveLocation(3)+rotationToDo,360);
         apRobot = setfield(apRobot,"nextLocation",nextLoc);         
 %         apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);
%          [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot);       % wait for updated information from robot  
                      % get and store BNO subsystem data 
          apRobot = setfield(apRobot,"subsytemLeft",[robot.BNOLeftPosX,robot.BNOLeftPosY,robot.BNOLocHeading]);
          apRobot = setfield(apRobot,"subsytemRight",[robot.BNORightPosX,robot.BNORightPosY,robot.BNOLocHeading]);  
              % compute locations   
          gyroBasedH=mod(apGet(apRobot,"gyroLocation")(3)+apGet(apRobot,"lastRotation"),360);
          gyroBasedX=round(apGet(apRobot,"gyroLocation")(1)+apGet(apRobot,"lastMove")*cos(gyroBasedH*pi()/180)+shiftEchoVsRotationCenter*cos(gyroBasedH*pi()/180));
          gyroBasedY=round(apGet(apRobot,"gyroLocation")(2)+apGet(apRobot,"lastMove")*sin(gyroBasedH*pi()/180)+shiftEchoVsRotationCenter*sin(gyroBasedH*pi()/180));
          gyroBasedX=gyroBasedX-shiftEchoVsRotationCenter*cos(gyroBasedH*pi()/180); % set position to rotation center
          gyroBasedY=gyroBasedY-shiftEchoVsRotationCenter*sin(gyroBasedH*pi()/180); % set position to rotation center
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