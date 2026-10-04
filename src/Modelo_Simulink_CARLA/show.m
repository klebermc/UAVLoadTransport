function [V]=show(gain,prob,c);
% function show(gain,prob);
% draws gain/prob graph;
V=plot(gain,prob,'r');
[numregions,tmp]=size(gain);
if (tmp>numregions)
	numregions=tmp;
end;
% numregions=numregions-1;
for i=1:1:numregions,
	l=line([gain(i) gain(i)],[0 prob(i)]);,
	if (nargin>2)
		VV=get(l,'Color');,
		set(l,'Color',c);,
	end;,
end;
if (ishold==0)
	hold on;
	if (nargin>2)
		plot(gain,prob,c);
	else
		plot(gain,prob);
	end;
	hold off;
else
	if (nargin>2)
		plot(gain,prob,c);
	else
		plot(gain,prob);
	end;
end;
%Comment
r=gain(numregions)-gain(1);
l=line([gain(1) gain(numregions)],[1/r 1/r]);

% Dispersion Coefficient & Peak Convergence Values
d=(density(0.75,gain,prob)-density(0.25,gain,prob))/(0.5*(gain(numregions)-gain(1)));
d=100-d*100;
p=max(prob)*(gain(numregions)-gain(1));

%Comment
disp(sprintf('%% Dispersion Coeff %5.2f Peak Convergence Value %5.2f \n',d,p));

[a,b]=max(prob);g=gain(b);

%Comment
disp(sprintf('Max Gain %8.2f \n',g));