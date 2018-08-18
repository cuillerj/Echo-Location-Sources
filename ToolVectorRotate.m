function [output] = ToolVectorRotate(loc,alpha)
  % plan positive rotation 
    R=[cos(alpha),-sin(alpha);sin(alpha),cos(alpha)];
    output=R*loc;
endfunction