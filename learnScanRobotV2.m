% lambda and maxIter wiil be used by oneVsAll()
function [] = learnScanRobot(lambdamin,lambdamax,maxIter)
%% Initialization
%% Setup the parameters 
input_layer_size  = 360;  % 
load 'extScanResult'
num_labels = max(extScanResult(:,5))          %  look in the data to find the features number
delta=lambdamax-lambdamin
% Load Training Data

load('trainMat.mat'); % load the training data
m = size(trainMat, 1);
load ('trainResult.mat');
Yv4=trainResult;
sY=size(Yv4)
lambda=lambdamin;


%% ============ Part 2: Vectorize Logistic Regression ============
%  use regularized logistic regression (implementation is vectorized).
%  one-vs-all classification 
% 
%
i=1;
while (lambda<=lambdamax)
	ctime (time ())
	fprintf('\nTraining One-vs-All Logistic Regression.%d..\n',lambda)
	[all_theta] = oneVsAll(trainMat, Yv4, num_labels, lambda,maxIter);
	fname=strcat("all_theta",mat2str(i),".mat");
	save  ("-mat4-binary",fname,"all_theta")
	%% ================ Part 3: Predict for One-Vs-All ================
	% calculate the accuaracy of the prediction with the training data
	load('trainMat.mat'); % training data stored in arrays X, y
	m = size(trainMat, 1);
	predTrain = predictOneVsAll(all_theta, trainMat); % return the predicted localization (zone/angle)
	accuracy(i,1)=lambda;
	accuracy(i,2)=mean(double(predTrain == Yv4)) * 100;
	save  ("-mat4-binary","accuracy.mat","accuracy")
	ctime (time ())
	fprintf('\nTraining Set Accuracy: %f\n', mean(double(predTrain == Yv4)) * 100);
	lambda=lambda+delta/20;
	i=i+1;
end

