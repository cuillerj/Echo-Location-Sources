   function [apRobot,robot,retCodeMove]=ApRobotMoveStraight(apRobot,robot,lenToDo,forward,plotOn)
                if (!exist("plotOn"))  % flat logistic regression is default mode 
                    plotOn=false;
                endif
                debugOn=1;
                printf(mfilename);
                printf(" move: %d  *** ",lenToDo);
                printf(ctime(time()));
                apRobot = setfield(apRobot,"currentMove","straig");
                robot.Move(0,lenToDo); 		 % len sent in cm
                lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
                saveLocation=apGet(apRobot,"saveLocation");
                if (lenToDo>=0)
                apRobot = setfield(apRobot,"forward",true); 
                 else
                apRobot = setfield(apRobot,"forward",false); 
                endif
                apRobot = setfield(apRobot,"lastMove",lenToDo);      
                [apRobot,robot]=ApMoveParticles(apRobot,robot,0,lenToDo,plotOn);
                apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
                [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
                if (retCodeMove>=99)  
                  [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCodeMove);
                  if action=="stop.."    % 
                    retCodeMove=-99;
                    return
                  endif
                  if action=="break."
                    break
                  endif
                  if action=="resume"
                    endif
                endif  
                probExpectedMoveOk=50;
                if (retCodeMove==robot.moveKoDueToWheelStopped )
                  newLenToDo=sqrt((robot.GetHardPosX()-robot.posX)^2+(robot.GetHardPosY()-robot.posY)^2)*forward;
                  printf(mfilename);
                  printf(" incompleted moveKoDueToWheelStopped expected: %d actual:%d %d %d %d %d *** ",lenToDo,newLenToDo,robot.GetHardPosX(),robot.posX,robot.GetHardPosY(),robot.posY);
                  printf(ctime(time()));
                  reseted=false;
                  while(!reseted)
                    robot.ResetRobotStatus();
                    pause(2);
                    if(robot.motorDiag==0 && robot.robotDiag==0)
                      reseted=true;
                    endif
                  endwhile                   
                  robot.Move(0,-sign(lenToDo)*apGet(apRobot,"backMoveWhenCollision")); 		 % len sent in cm
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  newLenToDo=newLenToDo-sign(lenToDo)*apGet(apRobot,"backMoveWhenCollision");
                  apRobot = setfield(apRobot,"lastMove",newLenToDo); 
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  [apRobot,robot]=ApMoveParticles(apRobot,robot,0,newLenToDo-sign(lenToDo)*apGet(apRobot,"backMoveWhenCollision"),plotOn);
                  apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
                  [apRobot,robot,retCodeMove]=ApWaitForRobot(apRobot,robot,debugOn);
                  if (retCodeMove>=99)  
                    [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCodeMove);
                    if action=="stop.."
                      return
                    endif
                    if action=="break."
                      break
                    endif
                    if action=="resume"
                    endif
                  endif  
                  probExpectedMoveOk=25;
 %                 gyroLenToDo=newLenToDo;
                endif	
                if (retCodeMove==robot.moveKoDueToObstacle )
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
                if (retCodeMove==robot.moveUnderLimitation)
                  printf(mfilename);
                  printf(" no move moveUnderLimitation. *** ")
                  printf(ctime(time()))
                  apRobot = setfield(apRobot,"lastMove",0);
                  apRobot = setfield(apRobot,"newTarget",1);              % to force new path calculation  
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  apRobot = setfield(apRobot,"location",apGet(apRobot,"saveLocation"));
                  probExpectedMoveOk=1;
                endif
                if (retCodeMove==robot.moveKoDueToNotEnoughSpace)
                  maxLen=robot.GetRetcodeDetail();
                  printf(mfilename);
                  printf(" no move moveKoDueToNotEnoughSpace Max possible len:% d *** ",maxLen);
                  printf(ctime(time()));
                  apRobot = setfield(apRobot,"particles",lastParticles);
                  apRobot = setfield(apRobot,"lastMove",0);
                  apRobot = setfield(apRobot,"location",apGet(apRobot,"saveLocation"));
                  probExpectedMoveOk=1;
                endif
                 if (retCodeMove==robot.moveWheelSpeedInconsistancy)
                  printf(mfilename);
                  printf(" pb move moveWheelSpeedInconsistancy  *** ");
                  printf(ctime(time()));
                  probExpectedMoveOk=1;
                endif
               apRobot = setfield(apRobot,"waitFor",robot.robotInfoUpdated);
               robot.requestBNOData();
               [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);       % wait for updated information from robot   
               robot.ValidHardPosition(); 
endfunction