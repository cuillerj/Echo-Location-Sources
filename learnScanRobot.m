% lambda and maxIter to use by oneVsAll()
function [] = learnScanRobot(lambda,maxIter)
%% Initialization
%% Setup the parameters 
input_layer_size  = 360;  % 
load 'extScanResult'
num_labels = max(extScanResult(:,5))          % 4

num_labels

% Load Training Data

load('trainMat.mat'); % training data
m = size(trainMat, 1);
load ('trainResult.mat');
Yv4=trainResult;
sY=size(Yv4)
		


%% ============ Part 2: Vectorize Logistic Regression ============
%  In this part of the exercise, you will reuse your logistic regression
%  code from the last exercise. You task here is to make sure that your
%  regularized logistic regression implementation is vectorized. After
%  that, you will implement one-vs-all classification for the handwritten
%  digit dataset.
%
%lambda = l
%maxIter=m;
fprintf('\nTraining One-vs-All Logistic Regression...\n')


[all_theta] = oneVsAll(trainMat, Yv4, num_labels, lambda,maxIter);
save  ("-mat4-binary","all_theta.mat","all_theta")
%fprintf('Program paused. Press enter to continue.\n');
%predpause;


%% ================ Part 3: Predict for One-Vs-All ================
%  After ...
load('trainMat.mat'); % training data stored in arrays X, y
m = size(trainMat, 1);
%load ('Yv4.csv');

%sY=size(Yv4)


predTrain = predictOneVsAll(all_theta, trainMat);

fprintf('\nTraining Set Accuracy: %f\n', mean(double(predTrain == Yv4)) * 100);

%load('trainVal.mat'); % validation data stored in arrays X, y
%m = size(trainVal, 1);
%load ('YVal.csv');
%sY=size(YVal)
%if num_labels ==3
%	for i=1:sY
%		if YVal(i)>=3
%			YVal(i)=YVal(i)-1;
%		end
%	end
%end
%predVal = predictOneVsAll(all_theta, trainVal);

%fprintf('\nTraining Set Accuracy: %f\n', mean(double(predVal == YVal)) * 100);

