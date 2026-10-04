function [Ex,var]=expcarla(gain,prob)
%
%       function [Ex,var]=expcarla(gain,prob)
%
%       Calculates the Expected value and variance of
%       a CARLA probability density function
%
[m, n] = size(gain);
numregions = max([m n]) - 1;
extmp = zeros(1,numregions);
ex2tmp = zeros(1,numregions);

g2=gain.*gain;
g3=gain.*g2;
g4=gain.*g3;

for i=1:1:numregions,

        grad = (prob(i+1) - prob(i)) / (gain(i+1) - gain(i));
        crossing = prob(i+1) - grad*gain(i+1);
        t4 = (g4(i+1) - g4(i)) / 4;
        t3 = (g3(i+1) - g3(i)) / 3;
        t2 = (g2(i+1) - g2(i)) / 2;

        extmp(i) = t3 * grad + t2 * crossing;
        ex2tmp(i) = t4 * grad + t3 * crossing;
end

Ex = sum(extmp);
Ex2 = sum(ex2tmp);
var = Ex2-Ex*Ex;



