function [angleOneHole,distanceOneHole] = StepEncoderByHole()
% tenir compte du nb entier de trous encodeurs dans rotation cost
RobotWidth = 46.5; 
WheelDiameter = 6.4;
WheelEncoderHoles=8;
distanceOneHole=0.0;
angleOneHole=(((pi*WheelDiameter)/WheelEncoderHoles)/(pi*RobotWidth))*360
distanceOneHole=(pi*WheelDiameter)/WheelEncoderHoles);
endfunction