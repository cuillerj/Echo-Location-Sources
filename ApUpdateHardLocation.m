function [apRobot,robot] = ApUpdateHardLocation(apRobot,robot,location,prob);
  robot.SetPosX(location(1));
  robot.SetPosY(location(2));
  robot.SetHeading(location(3));
  robot.SetCurrentLocProb(prob);
  while(robot.GetPosX()!=robot.GetHardPosX() || robot.GetPosY()!=robot.GetHardPosY() || mod(robot.GetHeading(),360)!=mod(robot.GetHardHeading(),360))
     robot.UpdateHardRobotLocation();
     WaitFor=robot.robotUpdatedEnd;
     apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);      
     [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot);
    
     if (retCode!=0)
         [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot);
         if action=="stop.."
             return
          endif
         if action=="break."
            break
         endif
         if action=="resume"
            resume
         endif
     endif
   end
   apRobot = setfield(apRobot,"location",location);
   printf(mfilename);
   printf(" hardPosX:%d hardPosY:%d prob:%d *** ",robot.GetHardPosX(),robot.GetHardPosY(),robot.GetCurrentLocProb());
   printf(ctime(time()))
 endfunction