function [apRobot,robot,retCode] = ApUpdateHardLocation(apRobot,robot,location,prob);
  robot.SetPosX(location(1));
  robot.SetPosY(location(2));
  robot.SetHeading(location(3));
  robot.SetCurrentLocProb(round(100*prob));
  retCode=0;
  while(robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360))
     robot.UpdateHardRobotLocation();
     WaitFor=robot.robotUpdatedEnd;
     apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);      
     [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot);
    
     if (retCode!=0)
         [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
         if action=="stop.."
             return
          endif
         if action=="break."
            break
         endif
         if action=="resume"
            return
         endif
     endif
   end
   apRobot = setfield(apRobot,"location",location);
   apRobot = setfield(apRobot,"locationProb",100*prob);   
   printf(mfilename);
   printf(" hardPosX:%d hardPosY:%d prob:%f *** ",robot.GetHardPosX(),robot.GetHardPosY(),robot.GetCurrentLocProb());
   printf(ctime(time()))
 endfunction