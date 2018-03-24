function [pred] = predictxVsAll(all_theta, X,x)
%PREDICT Predict the label for a trained one-vs-all classifier. The labels 
%are in the range 1..K, where K = size(all_theta, 1). 
%  p = PREDICTONEVSALL(all_theta, X,x) will return a vector of predictions containing the x most probable predictions
%  for each example in the matrix X. Note that X contains the examples in
%  rows. all_theta is a matrix where the i-th row is a trained logistic
%  regression theta vector for the i-th class. You should set p to a vector
%  of values from 1..K (e.g., p = [1; 3; 1; 2] predicts classes 1, 3, 1, 2
%  for 4 examples) 
%  

m = size(X, 1);
num_labels = size(all_theta, 1);

% You need to return the following variables correctly 
p = zeros(size(X, 1), 1);

% Add ones to the X data matrix
X = [ones(m, 1) X];

% ====================== YOUR CODE HERE ======================
% Instructions: Complete the following code to make predictions using
%               your learned logistic regression parameters (one-vs-all).
%               You should set p to a vector of predictions (from 1 to
%               num_labels).
%
% Hint: This code can be done all vectorized using the max function.
%       In particular, the max function can also return the index of the 
%       max element, for more information see 'help max'. If your examples 
%       are in rows, then, you can use max(A, [], 2) to obtain the max 
%       for each row.
%       
mat=X*all_theta';
% supprimer le save pour ameliorer perf
%save  ("-mat4-binary","mat.mat","mat")
[w j]=min(mat,[],2);
n=1;
while (n<=x)
	[v i]=max(mat,[],2);
	pred(n,1)=i;
	pred(n,2)=v;
	mat(i)=w-1;
	n=n+1;
end
pred(n,1)=j;
pred(n,2)=w;
% =========================================================================


end
