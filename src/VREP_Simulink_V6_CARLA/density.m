function [val]=density(percentage,gain,prob);
% function [val]=density(percentage,gain,prob);
% returns gain at a given percentage
% of the probability density
%[numregions,tmp]=size(gain);
%numregions=numregions-1;
area=0;
i = 0;
while (area<percentage(1)) & (i+1<length(prob)),
	i = i+1;
	f = 0.5*(gain(i+1)-gain(i))*(prob(i)+prob(i+1));
	area = area + f;
end;
% answer is between i & i+1
if (area==percentage(1)) 
	val = gain(i+1); 
elseif (prob(i+1)==prob(i))
	percentage = percentage - (area - f);
	val = percentage/prob(i)+gain(i);
else
	area = percentage - (area - f);
	tantheta = (prob(i+1)-prob(i))/(gain(i+1)-gain(i));
	c(1) = 0.5*tantheta;
	c(2) = prob(i) - gain(i)*tantheta;
	c(3) = 0.5*gain(i)^2*tantheta - gain(i)*prob(i) - area;
	if (c(2)^2>4*c(1)*c(3)) 
		root=roots(c);
		sol1=root(1);
		sol2=root(2);
	end;
	if ((prob(i)+(sol1-gain(i))*tantheta)<0.0)
		sol1=sol2;
	elseif ((prob(i)+(sol2-gain(i))*tantheta)<0.0)
		sol2=sol1;
	end;
	if ((sol1>=gain(i)) & (sol1<=gain(i+1)))
        	val = sol1;
	else
		val = sol2;
	end;
end;
