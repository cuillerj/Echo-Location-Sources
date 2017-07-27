function [] = ApShowComparedStep(apRobot,step1,step2,head) 
%{
 plot the step1s of a move
 step1 is a matrix containinf the [(x,y)] points
 head is the title of the graph
 %}
shitfCartoX=apGet(apRobot,"shitfCartoX");
shitfCartoY=apGet(apRobot,"shitfCartoY")
currentL=apGet(apRobot,"location");

figure()
title (head);

%axis([1,b,1,a],"on","ij");

load carto1img;
img=carto1img;
[a,b]=size(img);
hold on;
imshow(img,[])
%axis([1-shitfCartoX,b,-1-shitfCartoY,a],"on","xy");
%axis([1,b,1,a],"square","on","xy");
axis("image","square","on","xy");
ylab=strcat("y position + ",num2str(shitfCartoY)),
ylabel(ylab);
xlab=strcat("X position + ",num2str(shitfCartoX)),
xlabel(xlab);
%plot(currentL(1)+shitfCartoX,currentL(2)+shitfCartoY,"k:x")
i=0;
for i=1:size(step1,1)
%	x=x+step1(i,1);
%	y=y+step1(i,2);
%	plot(step1(i,1),a+1-step1(i,2),"r")
  if (mod(i,2)==0)
    line ([step1(i,1)+shitfCartoX step1(i,1)+shitfCartoX+15*cos(step1(i,3)*pi()/180)], [step1(i,2)+shitfCartoY step1(i,2)+shitfCartoY+15*sin(step1(i,3)*pi/180)], "linestyle", "-", "linewidth",1.5,"color", "c");
	  plot(step1(i,1)+shitfCartoX,step1(i,2)+shitfCartoY,"markersize",10,"b:+")
	%  ax=plotyy(1,1,step1(i,1),step1(i,2))
    else
  line ([step1(i,1)+shitfCartoX step1(i,1)+shitfCartoX+15*cos(step1(i,3)*pi()/180)], [step1(i,2)+shitfCartoY step1(i,2)+shitfCartoY+15*sin(step1(i,3)*pi/180)], "linestyle", "-", "linewidth",1.5,"color", "b");    
 	  plot(step1(i,1)+shitfCartoX,step1(i,2)+shitfCartoY,"markersize",10,"c:*") 
  endif

endfor

for i=1:size(step2,1)
%	x=x+step1(i,1);
%	y=y+step1(i,2);
%	plot(step1(i,1),a+1-step1(i,2),"r")
  if (mod(i,2)==0)
   line ([step2(i,1)+shitfCartoX step2(i,1)+shitfCartoX+15*cos(step2(i,3)*pi()/180)], [step2(i,2)+shitfCartoY step2(i,2)+shitfCartoY+15*sin(step2(i,3)*pi/180)], "linestyle", "-", "linewidth",1.5,"color", "r");
	  plot(step2(i,1)+shitfCartoX,step2(i,2)+shitfCartoY,"markersize",10,"k:+")
  else
   line ([step2(i,1)+shitfCartoX step2(i,1)+shitfCartoX+15*cos(step2(i,3)*pi()/180)], [step2(i,2)+shitfCartoY step2(i,2)+shitfCartoY+15*sin(step2(i,3)*pi/180)], "linestyle", "-", "linewidth",1.5,"color", "k");
 	  plot(step2(i,1)+shitfCartoX,step2(i,2)+shitfCartoY,"markersize",10,"r:*") 
  endif
endfor
grid minor;
hold off
%{
load "imageCaseCuisine.jpg"
imshow("imageCaseCuisine.jpg",[])

[a,b]=size(imageCaseCuisine);
for i=1:size(step1,1)
%	x=x+step1(i,1);
%	y=y+step1(i,2);
	plot(step1(i,1),a+1-step1(i,2),"r")
endfor
hold off
%}
endfunction