function [prob] = Gaussian2D(mu1,sigma1,x1,mu2,sigma2,x2)
# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
   %   prob= exp(- (((mu1 - x1) ^2) / (sigma1 ^2) +((mu2 - x2) ^2) / (sigma2 ^2) / 2.0 / (sqrt(2.0 * pi * (sigma1 ^2))+sqrt(2.0 * pi * (sigma2 ^2)))
  prob= exp(- ((mu1 - x1) ^2 / (sigma1 ^2) +(mu2 - x2) ^2) / (sigma2 ^2) ) 
		
endfunction