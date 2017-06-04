function [carto1] = ApCreateCartoMatrix(name)
% load carto en format txt et save au format matrice
strfin='.txt';
str=strcat(name,strfin);
X=load (str);
X=X';
X=fliplr(X);
[x,y]=size(X);
carto1=zeros(10*x,10*y);
carto2=zeros(10*x,10*y);
for i=0:x-1
	for j=0:y-1
		init=X(i+1,j+1);
			for k=10*i+1:10*i+11
				for l=10*j+1:10*j+11
					carto1(k,l)=init;
					carto2(k,l)=mod(init+56,256);
				endfor
			endfor
	endfor
endfor
save ("-mat4-binary","carto1.mat","carto1")
printf("created: carto1.mat\n");
carto1img=flipud(rot90(carto2));
strfin='.mat';
matrix=strcat(name,strfin);
%name=carto1mg;
save ("-mat4-binary","carto1img.mat","carto1img")
clf
imshow(carto1img,[])
endfunction


