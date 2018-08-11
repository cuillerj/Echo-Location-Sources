function [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOn)
                aligned=false;
                action="......";
                lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
%                WaitMove=robot.moveEnd;
 %               WaitNorthAlign=robot.northAlignEnd;
                debugOn=1;
                apRobot = setfield(apRobot,"lastRotation",rotationToDo);
                apRobot = setfield(apRobot,"saveLocation",apGet(apRobot,"location"));
                apRobot = setfield(apRobot,"saveGyroLocation",apGet(apRobot,"gyroLocation"));
                apRobot = setfield(apRobot,"currentMove","rotate");
                callFrom=apGet(apRobot,"callFrom");
                if (rotationType==1)
                  saveNO=robot.northOrientation;
                  printf(mfilename);
                  printf(" NO rotate robot NO: %d rotation:%d  *** ",robot.northOrientation,rotationToDo)
                  printf(ctime(time()));									
                  robot.RobotNorthRotate(mod(round(rotationToDo)+360,360));    % rotation based on north orientation
                  apRobot = setfield(apRobot,"waitFor",robot.northAlignEnd);
 %                 WaitFor=WaitNorthAlign;
                endif
                if (rotationType==2)
                  printf(mfilename);
                  printf(" gyro rotate robot  rotation:%d  *** ",rotationToDo)
                  printf(ctime(time()));						
                  robot.RobotGyroRotate(round(rotationToDo));    % rotation based gyroscope
%                  WaitFor=WaitMove;
                  apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
                endif
                if (rotationType==3)
                  printf(mfilename);
                  printf(" wheels rotate robot  rotation:%d  *** ",rotationToDo)
                  printf(ctime(time()));							
                  robot.Move(round(rotationToDo),0);    % rotation based on wheels encoders
                  apRobot = setfield(apRobot,"waitFor",robot.moveEnd);
                endif
                [apRobot,robot]=ApMoveParticles(apRobot,robot,round(rotationToDo),0,plotOn);       
                [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot,debugOn);
                if (retCode==-99)
                     action="stop..";
                     return;                 
                endif
                if (retCode==0)
                  aligned=true;
                else
                    if (retCode==robot.moveKoDueToNotEnoughSpace)
                          printf( mfilename);
                          printf(" no rotation moveKoDueToNotEnoughSpace. *** ");
                          printf(ctime(time()));
                          apRobot = setfield(apRobot,"lastRotation",0);
                          apRobot = setfield(apRobot,"particles",lastParticles);  % restaure particles
                          apRobot = setfield(apRobot,"location",apGet(apRobot,"saveLocation"));
                          apRobot = setfield(apRobot,"gyroLocation",apGet(apRobot,"saveGyroLocation"));
                          action="retry.";
                     else
                          [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
                          if (action=="stop..")
                            return;
                          endif
                          if (action=="inMove")
                            return;
                          endif
                     endif
                     reseted=false;
                     while(!reseted)
                        robot.ResetRobotStatus();
                        pause(2);
                        if(robot.motorDiag==0 && robot.robotDiag==0)
                          reseted=true;
                        endif
                      endwhile    
                endif
               apRobot = setfield(apRobot,"waitFor",robot.robotUpdatedEnd);
               [apRobot,robot,retCode] = ApWaitForRobot(apRobot,robot,debugOn);       % wait for updated information from robot
                retry=0;
                robot.ValidHardPosition(); 
               apRobot = setfield(apRobot,"gyroLocation",[apGet(apRobot,"gyroLocation")(1),apGet(apRobot,"gyroLocation")(2),robot.GetGyroHeading()]);
               while (robot.BNOLocFlag!=0 && retry<=3)
                   pause(1);
                   retry=retry+1;
                   robot.ValidHardPosition();          % the new hard heading will be taken into account by java code
                   apRobot = setfield(apRobot,"gyroLocation",[apGet(apRobot,"gyroLocation")(1),apGet(apRobot,"gyroLocation")(2),robot.GetGyroHeading()]);
               end
%{               
              if (rotationType==1 && aligned==true)
                robot.SetHeading(mod(360+robot.GetHardHeading()+robot.northOrientation-saveNO,360));
              endif
              if (rotationType==2 && aligned==true)
                robot.SetHeading(mod(360+apGet(apRobot,"saveLocation")(3)+robot.GetGyroHeading(),360));
                gyroLocation=[apGet(apRobot,"gyroLocation")(1),apGet(apRobot,"gyroLocation")(2),apGet(apRobot,"gyroLocation")(3)+robot.GetGyroHeading()];
                apRobot = setfield(apRobot,"gyroLocation",gyroLocation);
              endif
              if (rotationType==3 && aligned==true)
                robot.SetHeading(mod(360+robot.GetHardHeading(),360));
              endif
%}
  endfunction