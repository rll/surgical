function [dG, G] = GaussianDerivative(sig)
%% Computes derivative of a Gaussian
%% Automatically computes for x = -(3*sig) to x = (3*sig)
%% -x/(sig^3/sqrt(2pi)) exp(-x^2/(2sig^2))
%%
x = floor(-3*sig):1:ceil(3*sig);
G = exp( (-x.*x)/(2*sig*sig) );
G = G / (sig * sqrt(2*pi));

dG = (-x/(sig*sig)) .* G;
