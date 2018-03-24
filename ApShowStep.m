function [] = ApShowStep(apRobot,step,head,figureNb) 
%{
 plot the steps of a move
 step is a matrix containinf the [(x,y)] points
 head is the title of the graph
 %}
shitfCartoX=apGet(apRobot,"shitfCartoX");
shitfCartoY=apGet(apRobot,"shitfCartoY");
currentL=apGet(apRobot,"location");
step;
if (!exist("figureNb"))  % flat or rotated IA echo location 
     figure();
     title (head);
%     load carto1img;
 %    img=carto1img;
     	img=apGet(apRobot,"img");
      [a,b]=size(img);
      c=max(a,b);
      hold on;
      imshow(img,[]);
    %axis([1-shitfCartoX,b,-1-shitfCartoY,a],"on","xy");
      axis([1,c,1,c],"square","on","xy");
      plot(currentL(1)+shitfCartoX,currentL(2)+shitfCartoY,"k:x")
else
  figure(figureNb); 
  hold on;
  plot(currentL(1)+shitfCartoX,currentL(2)+shitfCartoY,"k:x")  
endif
%axis([1,b,1,a],"on","ij");
i=0;
for i=1:size(step,1)-1
%	x=x+step(i,1);
%	y=y+step(i,2);
%	plot(step(i,1),a+1-step(i,2),"r")
  if (mod(i,2)==0)
	  plot(step(i,1)+shitfCartoX,step(i,2)+shitfCartoY,"r:x")
  else
 	  plot(step(i,1)+shitfCartoX,step(i,2)+shitfCartoY,"y:x") 
  endif
endfor
i=i+1;
plot(step(i,1)+shitfCartoX,step(i,2)+shitfCartoY,"g:x")
if (!exist("figureNb"))
  grid minor;
  hold off
  else
  hold on;
endif
%{
load "imageCaseCuisine.jpg"
imshow("imageCaseCuisine.jpg",[])

[a,b]=size(imageCaseCuisine);
for i=1:size(step,1)
%	x=x+step(i,1);
%	y=y+step(i,2);
	plot(step(i,1),a+1-step(i,2),"r")
endfor
hold off
%}
endfunction