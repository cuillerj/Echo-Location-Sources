function [robot] = LoopPingFB(robot,count)
   if (!exist("count"))  % flat logistic regression is default mode 
       count=50;
    endif
  for i=0:count
    robot.PingEchoFrontBack();
    pause(10);
  end
endfunction