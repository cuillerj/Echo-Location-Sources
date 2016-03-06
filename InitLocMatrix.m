function [predLocMatrix] = InitLocMatrix(all_theta)
probInit=1/size(all_theta,1);
predLocMatrix=zeros(size(all_theta,1),1);
for i=1:size(all_theta,1)
	predLocMatrix(i)=probInit;
end
