% lambda and maxIter wiil be used by oneVsAll()
function [] = learnScanRobot(lambda,maxIter)
%% Initialization
%% Setup the parameters 
input_layer_size  = 360;  % 
load 'extScanResult'
num_labels = max(extScanResult(:,5))          %  look in the data to find the features number

% Load Training Data

load('trainMat.mat'); % load the training data
m = size(trainMat, 1);
load ('trainResult.mat');
Yv4=trainResult;
sY=size(Yv4)
		


%% ============ Part 2: Vectorize Logistic Regression ============
%  use regularized logistic regression (implementation is vectorized).
%  one-vs-all classification 
% 
%

fprintf('\nTraining One-vs-All Logistic Regression...\n')


[all_theta] = oneVsAll(trainMat, Yv4, num_labels, lambda,maxIter);
save  ("-mat4-binary","all_theta.mat","all_theta")

%% ================ Part 3: Predict for One-Vs-All ================
% calculate the accuaracy of the prediction with the training data
load('trainMat.mat'); % training data stored in arrays X, y
m = size(trainMat, 1);

predTrain = predictOneVsAll(all_theta, trainMat); % return the predicted localization (zone/angle)

fprintf('\nTraining Set Accuracy: %f\n', mean(double(predTrain == Yv4)) * 100);


