function [apRobot,robot,retCode] = ApMoveAcrossNarrowPass(apRobot,robot,passDistance,passWidth,lenToDo)
  printf(mfilename);
  printf(" move across pass: %d  *** ",lenToDo);
  printf(ctime(time()));
  robot.MoveAcrossNarrowPass(passDistance,passWidth,lenToDo)
  
                
                
  lastParticles=apGet(apRobot,"particles");          % to be able to recover in case of move failure   
  saveLocation=apGet(apRobot,"saveLocation");
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
endfunction