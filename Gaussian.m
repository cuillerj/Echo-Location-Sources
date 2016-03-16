function [prob] = Gaussian(mu,sigma,x)
# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        prob= exp(- ((mu - x) ^2) / (sigma ^2) / 2.0) / sqrt(2.0 * pi * (sigma ^2))
endfunction