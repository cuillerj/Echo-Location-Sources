function [apRobot,robot,retCode,action]=ApRobotRotate(apRobot,robot,rotationToDo,rotationType,plotOn)
                printf(ctime(time()))
                aligned=false;
                action="......";
                lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
%                WaitMove=robot.moveEnd;
 %               WaitNorthAlign=robot.northAlignEnd;
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
                [apRobot,robot]=ApMoveParticles(apRobot,robot,rotationToDo,0,plotOn);       
                [apRobot,robot,retCode]=ApWaitForRobot(apRobot,robot);

                if (retCode==0)
                  aligned=true;
                else
                    if (retCode==robot.moveKoDueToNotEnoughSpace)
                          printf( mfilename);
                          printf("no rotation moveKoDueToNotEnoughSpace. *** ")
                          printf(ctime(time()))
                          apRobot = setfield(apRobot,"particles",lastParticles);  % restaure particles
                     else
                          [apRobot,robot,issue,action]=ApAnalyseRetcode(apRobot,robot,retCode);
                          if (action=="stop..")
                            return;
                          endif
                     endif
                endif
              if (rotationType==1 && aligned==true)
                robot.SetHeading(mod(360+robot.GetHardHeading()+robot.northOrientation-saveNO,360));
              endif
              if (rotationType==2&& aligned==true)
                robot.SetHeading(mod(360+apGet(apRobot,"saveLocation")(3)+robot.GetGyroHeading(),360));
                gyroLocation=apGet(apRobot,"gyroLocation")+robot.GetGyroHeading()*pi()/180;
                apRobot = setfield(apRobot,"gyroLocation",gyroLocation);
              endif
              if (rotationType==3&& aligned==true)
                robot.SetHeading(mod(360+robot.GetHardHeading(),360));
              endif

  endfunction