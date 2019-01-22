function [apRobot,robot,figureNumber] = ApDrawRobot(apRobot,robot,clear,figureNumber);
  % load carto1img;
  % img=carto1img;
   img=apGet(apRobot,"img");
  [a,b]=size(img);
    if (!exist("figureNumber")||figureNumber==0) 
      keepFigure=false;
      figure();
      grid minor on ;
      imshow(img,[]);
      axis("square");
      axis equal;
      axis([1,b+20,1,a+20],"on","xy");
      grid minor on ;
      hold on
    else
      keepFigure=true;
      figure(figureNumber);
      if (clear==1)
        grid minor on ;
        imshow(img,[]);
        axis("square");
        axis equal;
        axis([1,b+20,1,a+20],"on","xy");
        grid minor on ;
      endif
      hold on;
    endif
  figureNumber=get (0, "currentfigure");
  hold on;
  iRobotWidth=apGet(apRobot,"iRobotFrontWidth");
  robotRadius=iRobotWidth/2;
  frontLenght=apGet(apRobot,"frontLenght");
  backLenght=apGet(apRobot,"backLenght");
  robotLenght=apGet(apRobot,"frontLenght")+ apGet(apRobot,"backLenght");
  shitfCartoX=apGet(apRobot,"shitfCartoX");
  shitfCartoY=apGet(apRobot,"shitfCartoY");;
  loc=apGet(apRobot,"location");
  plot(loc(1)+shitfCartoX,loc(2)+shitfCartoY,'color','r','marker','*','markersize',15);
  theta=loc(3)*pi()/180;
  frontX=frontLenght*cos(theta)+robotRadius*sin(theta);
  points=[loc(1)+frontLenght*cos(theta)+robotRadius*sin(theta),loc(2)+frontLenght*sin(theta)-robotRadius*cos(theta);
          loc(1)+frontLenght*cos(theta)-robotRadius*sin(theta),loc(2)+frontLenght*sin(theta)+robotRadius*cos(theta);
          loc(1)-backLenght*cos(theta)+robotRadius*sin(theta),loc(2)-backLenght*sin(theta)-robotRadius*cos(theta);
          loc(1)-backLenght*cos(theta)-robotRadius*sin(theta),loc(2)-backLenght*sin(theta)+robotRadius*cos(theta)
  ];
  for (i=3:4)
    plot(points(i,1)+shitfCartoX,points(i,2)+shitfCartoY,'color','g','marker','o','markersize',15);
  endfor
  backX=backLenght*cos(theta);
  frontY=frontLenght*sin(theta);
  backY=backLenght*sin(theta);
  rob=drawRect(points(3,1)+shitfCartoX,points(3,2)+shitfCartoY,robotLenght,iRobotWidth,loc(3));
  set (rob, "color", [1 1 0]); # set to red (RGB triplets)
  set (rob, "linewidth", 2);
 % rob=drawRect(loc(1)+shitfCartoX,loc(2)+shitfCartoY,robotLenght,iRobotWidth,loc(3))
  set (rob, "color", [1 0 0]); # set to red (RGB triplets)
  set (rob, "linewidth", 2);
endfunction