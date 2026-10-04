function [gain,prob] = init_cla(min,max,numregions);
% function [gain,prob] = init_cla(min,max,numregions);
% Initialise the continuous learning automata matrices
range = max-min;
gain=zeros(numregions+1,1);
prob=zeros(numregions+1,1);
for i=1:1:numregions+1,
	gain(i)=min+(i-1)*range/numregions;
	prob(i)=1/range;
end;
