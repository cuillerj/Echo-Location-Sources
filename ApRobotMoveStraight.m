   function [apRobot,robot,retCode]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOn)
                if (!exist("plotOn"))  % flat logistic regression is default mode 
                    plotOn=false;
                endif
                printf(mfilename);
                printf(" move: %d  *** ",lenToDo);
                printf(ctime(time()));
                robot.Move(0,lenToDo); 		 % len sent in cm
                lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
                saveLocation=apGet(apRobot,"saveLocation");
                apRobot = setfield(apRobot,"lastMove",lenToDo);      
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
                probExpectedMoveOk=50;
                if (retCode==robot.moveKoDueToWheelStopped )
                  newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
                  printf(mfilename);
                  printf(" incompleted moveKoDueToWheelStopped expected: %d actual:%d %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  apRobot = setfield(apRobot,"lastMove",newLenToDo);   
                  [apRobot,robot]=ApMoveParticles(apRobot,robot,0,newLenToDo,plotOn);
                  probExpectedMoveOk=25;
 %                 gyroLenToDo=newLenToDo;
                endif	
                if (retCode==robot.moveKoDueToObstacle )
                  newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
                  printf(mfilename);
                  printf(" incompleted move due to obstacle ! Expected dist: %d actual:%d  %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY)
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  apRobot = setfield(apRobot,"lastMove",newLenToDo);                       
                  [apRobot,robot]=ApMoveParticles(apRobot,robot,0,newLenToDo,plotOn);
                  probExpectedMoveOk=25;				
  %                gyroLenToDo=newLenToDo;
                endif														
                if (retCode==robot.moveUnderLimitation)
                  printf(mfilename);
                  printf(" no move moveUnderLimitation. *** ")
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"lastMove",0);
                  apRobot = setfield(apRobot,"newTarget",1);              % to force new path calculation  
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  apRobot = setfield(apRobot,"location",apGet(apRobot,"saveLocation"));
                  probExpectedMoveOk=1;
                endif
                if (retCode==robot.moveKoDueToNotEnoughSpace)
                  printf(mfilename);
                  printf(" no move moveKoDueToNotEnoughSpace. *** ")
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  apRobot = setfield(apRobot,"lastMove",0);
                  apRobot = setfield(apRobot,"newTarget",1);   
                  apRobot = setfield(apRobot,"location",apGet(apRobot,"saveLocation"));
                  probExpectedMoveOk=1;
                endif
               apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);
               [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot);       % wait for updated information from robot   

endfunction