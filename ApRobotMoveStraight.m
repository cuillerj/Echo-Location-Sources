   function [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOn)
                robot.Move(0,lenToDo); 		 % len sent in cm
                lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
                [apRobot,robot]=ApMoveParticles(apRobot,robot,0,lenToDo,plotOn);
                apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
                [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot);
                if (retCode>=99)  
                  [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
                  if action=="stop.."
                    return
                  endif
                  if action=="break."
                    break
                  endif
                  if action=="resume"
                    endif
                endif  
  %							robot.GetGyroHeading

                probExpectedMoveOk=50;
                if (retCode==robot.moveKoDueToWheelStopped )
                  newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
                  printf("incompleted moveKoDueToWheelStopped expected: %d actual:%d %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  [apRobot,robot]=ApMoveParticles(apRobot,robot,0,newLenToDo,plotOn);
                  probExpectedMoveOk=25;
                  gyroLenToDo=newLenToDo;
                endif	
                if (retCode==robot.moveKoDueToObstacle )
                  newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
                  printf("incompleted move due to obstacle ! Expected dist: %d actual:%d  %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  [apRobot,robot]=ApMoveParticles(apRobot,robot,0,newLenToDo,plotOn);
                  probExpectedMoveOk=25;				
                  gyroLenToDo=newLenToDo;
                endif														
                if (retCode==robot.moveUnderLimitation)
                  printf("no move moveUnderLimitation. *** ")
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  probExpectedMoveOk=1;
                endif
                if (retCode==robot.moveKoDueToNotEnoughSpace)
                  printf("no move moveKoDueToNotEnoughSpace. *** ")
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  probExpectedMoveOk=1;
                endif
                if (robot.BNOLocFlag==0)
                  robot.ValidHardPosition();
                else
                  pause(1);
                  robot.ValidHardPosition();
                endif