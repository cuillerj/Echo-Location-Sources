% create a matrix for one scan rotated by "rotation" degres
% 
function [] = createMatrixAnalyseWithRotationById(Id,rotation)
rotation
nbMesurementByTrain=getNbStepsRotation();
strdeb='ToAnalyse\';
strfin='.txt';
str=strcat(strdeb,Id,strfin);
str
copyfile (str, 'scanToAnalyse.txt','f');
load ('scanToAnalyse.txt');
nbRec=size(scanToAnalyse,1);
analyseMat=zeros(1,362);
analyseMatRotated=zeros(1,362);
j=1;
i=1;
	pixelBF=zeros(2,181);
		while (j<=nbRec)
				angle=scanToAnalyse(j,1);
				k=angle;
				pixelBF(1,k+1)=scanToAnalyse(j,2);
				pixelBF(2,k+1)=scanToAnalyse(j,3);

				if (j < nbRec)
					angleNext=scanToAnalyse(j+1,1);
					interval=1;
						while (k < angleNext )
							pixelBF(1,k+2)=scanToAnalyse(j,2)+(scanToAnalyse(j+1,2)-scanToAnalyse(j,2))/(angleNext-angle)*interval; 
							pixelBF(2,k+2)=scanToAnalyse(j,3)+(scanToAnalyse(j+1,3)-scanToAnalyse(j,3))/(angleNext-angle)*interval;
							if (scanToAnalyse(j,2)==0 || scanToAnalyse(j+1,2)==0)  
								pixelBF(1,k+2)=0;
							end
							if (scanToAnalyse(j,3)==0 || scanToAnalyse(j+1,3)==0)
								pixelBF(2,k+2)=0;
							end
							k=k+1;
							interval=interval+1;
						end
				end
				j=j+1;


		end
		pixelBF;
		save  ("-mat4-binary","pixelBF.mat","pixelBF");
		analyseMat(1,:)=reshape (pixelBF,1,[]);
		save  ("-mat4-binary","analyseMat.mat","analyseMat");
		pixelBFTr=pixelBF';
for i=(1:rotation)
	pixelBFTr=circshift(pixelBFTr,1);
	x=pixelBFTr(1,1);
	y=pixelBFTr(1,2);
	pixelBFTr(1,1)=y;
	pixelBFTr(1,2)=x;
endfor
	pixelBF=pixelBFTr';
analyseMatRotated(1,:)=reshape (pixelBF,1,[]);
size(analyseMatRotated);
save  ("-mat4-binary","analyseMatRotated.mat","analyseMatRotated")