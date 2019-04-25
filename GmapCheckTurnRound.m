function [apRobot,possible,rotation,step] = GmapCheckTurnRound(apRobot,expectedHeading)
    %{
    test if it is  securly possible to do  turn round  at the current location  
    if possible return rotation to do
    else find the closest possible position  to do so
    %}
    possible=false;
    rotation=0;
    step=[];
    currentL=apGet(apRobot,"location");
    carto=apGet(apRobot,"carto");
    shitfCartoX=apGet(apRobot,"shitfCartoX");
    shitfCartoY=apGet(apRobot,"shitfCartoY");
    
    cartoValue = carto(currentL(1)+shitfCartoX,currentL(2)+shitfCartoY)
    
    endfunction