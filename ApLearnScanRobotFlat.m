% lambda and maxIter wiil be used by oneVsAll()
function [] = ApLearnScanRobotFlat(lambdamin,lambdamax,maxIter,nbTry)
  %{
  lambdamin and lambdamax define the range of lambda to be tried
  maxIter define the maximum number of iterations of the logistic regression
  nbTry define the number of tries inside le lambda range - if lambdamin=lambdamax force to 1
  
  %}
%% Initialization
%% Setup the parameters 
  printf(mfilename);
                  printf(ctime(time()));
delta=lambdamax-lambdamin
if (delta==0)
  nbTry=1;
  delta=1;
endif
input_layer_size  = 360;  % 
load 'scanResult.txt'
num_labels = max(scanResult(:,5))          %  look in the data to find the features number

% Load Training Data

load('trainMatFlat.mat'); % load the training data
m = size(trainMatFlat, 1);
load ('trainResultFlat.mat');
Yv4=trainResultFlat;
sY=size(Yv4)
lambda=lambdamin;
testFlatResult=[];

%% ============ Part 2: Vectorize Logistic Regression ============
%  use regularized logistic regression (implementation is vectorized).
%  one-vs-all classification 
% 
%
i=1;
while (lambda<=lambdamax)
	ctime (time ())
	fprintf('\nTraining One-vs-All Logistic Regression.%d MaxIter %d labels Number %d..\n',lambda,maxIter,num_labels)
	[all_thetaFlat] = oneVsAll(trainMatFlat, Yv4, num_labels, lambda,maxIter);
	fname=strcat("all_thetaFlat",mat2str(i),".mat");
	save  ("-mat4-binary",fname,"all_thetaFlat")
	save  ("-mat4-binary","all_thetaFlat.mat","all_thetaFlat")
	%% ================ Part 3: Predict for One-Vs-All ================
	% calculate the accuaracy of the prediction with the training data
	load('trainMatFlat.mat'); % training data stored in arrays X, y
	m = size(trainMatFlat, 1);
	predTrain = predictOneVsAll(all_thetaFlat, trainMatFlat); % return the predicted localization (zone/angle)
	accuracyFlat(i,1)=lambda;
	accuracyFlat(i,2)=mean(double(predTrain == Yv4)) * 100;
	save  ("-mat4-binary","accuracyFlat.mat","accuracyFlat")
	ctime (time ())
	fprintf('\nTraining Set accuracyFlat: %f\n', mean(double(predTrain == Yv4)) * 100);
	% test
	[coeff,foundFirst,notFound,nbrec] = ApEvaluateLearningResultsFlat()
	testFlatResult=[testFlatResult;[lambda,accuracyFlat(i,2),coeff,foundFirst,notFound,nbrec,maxIter,m,num_labels]];
	save  ("-mat4-binary","testFlatResult.mat","testFlatResult")
	%
	lambda=lambda+round(delta/nbTry);
	i=i+1;
end

