distance=180;
sigmaDist=20;
min=distance-4*sigmaDist;
if (min<1)
  min=1;
endif
max=distance+4*sigmaDist;
nb=100000;
m=zeros(1,50*sigmaDist);
for i=1:nb
  value=round(normrnd(distance,sigmaDist));
  if (value < size(m,2) && value >0)
    m(value)=m(value)+1;
  endif
endfor
f=figure();
h=bar([min:max],m(min:max),0.5);
set (h(1), "facecolor", "g")
hy=0.02*nb;
line ([distance,distance] ,[0 hy], "linestyle", "-", "color", "r","linewidth",2);
line ([distance-sigmaDist,distance-sigmaDist] ,[0 hy], "linestyle", "-", "color", "b","linewidth",2);
line ([distance+sigmaDist,distance+sigmaDist] ,[0 hy], "linestyle", "-", "color", "b","linewidth",2);
count=0;
for i=distance-sigmaDist:distance+sigmaDist
  count=count+m(i);
endfor
count/nb