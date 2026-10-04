function [gain,prob]=mep(gain,prob);
% function[gain,prob]=mep(gain,prob);
% Continuous Learning Automata 
% Moving End Points Code
% Free Parameters
reduction = 0.17; 
expansion = 0.01;
twentyfive = density(0.25,gain,prob);
seventyfive = density(0.75,gain,prob);
[numregions,tmp]=size(gain);
tmpprob=zeros(numregions,1);
tmpgain=zeros(numregions,1);
numregions=numregions-1;
xrange = gain(numregions+1)-gain(1);
if (twentyfive> ( gain(1) + (0.25+reduction)*xrange))
% reduce range
	sgain = gain(1) + (twentyfive-gain(1))/100;
elseif (twentyfive < ( gain(1) + (0.25-expansion)*xrange))
% increase range
	sgain = gain(1)-(twentyfive-gain(1))/100;
else
	sgain =gain(1);
	sprob=prob(1);
end;
if (seventyfive<(gain(1)+(0.75-reduction)*xrange))
% reduce range
	fgain = gain(numregions+1)-(gain(numregions+1)-seventyfive)/100;
elseif (seventyfive>(gain(1)+(0.75+expansion)*xrange))
% increase range
	fgain = gain(numregions+1)+(gain(numregions+1)-seventyfive)/100;
else
	fgain = gain(numregions+1);
	fprob = prob(numregions+1);
end;
if (sgain==gain(1))
	sprob=prob(1);
	n = 1;
elseif (sgain<gain(1))
	sprob=prob(1);
	n=1;
else
	i=1;
	while (sgain>gain(i))
		i=i+1;
	end;
	if (sgain==gain(i-1))
		sprob=prob(i);
	elseif (prob(i)==prob(i-1))
		sprob=prob(1);
	else
		tantheta = (prob(i)-prob(i-1))/(gain(i)-gain(i-1));
		sprob = prob(i-1)+(sgain-gain(i-1))*tantheta;
	end;
	n=i-1;
end;
gain(n)=sgain;
prob(n)=sprob;
if (fgain==gain(numregions+1))
	fprob=prob(numregions+1);
	m=numregions;
elseif (fgain>gain(numregions+1))
	fprob=prob(numregions+1);
	m=numregions;
else
	i=numregions+1;
	while (fgain<gain(i)) 
		i=i-1;
	end;
	if (fgain==gain(i))
		fprob=prob(i);
	elseif (prob(i)==prob(i+1))
		fprob=prob(i);	
	else
		tantheta = (prob(i+1)-prob(i))/(gain(i+1)-gain(i));
		fprob = prob(i)+(fgain-gain(i))*tantheta;
	end;
	m = i;
end;
gain(m+1)=fgain;
prob(m+1)=fprob;
%if ((n~=1) | (m~=numregions))
new_area=0;
for i=n:1:m,
	new_area = new_area+0.5*(gain(i+1)-gain(i))*(prob(i)+prob(i+1));
end;
new_area=new_area/numregions;
tmpprob(1)=prob(n);
tmpgain(1)=gain(n);
xr=gain(n);
pr=prob(n);
req_area=new_area;
i=n;
k=1;
check=0.0;
while ((i<=m) & (k<=numregions))
	xa=xr;
	pa=pr;
	xb=gain(i+1);
	pb=prob(i+1);
	check = check+0.5*(xb-xr)*(pb+pa);
	if (check<new_area)
		req_area = req_area - 0.5*(xb-xa)*(pa+pb);
		xr=xb;
    		pr=pb;
		i = i+1;
	elseif (pb==pa)
		sol1= (req_area/pa) + xa;
		if (check>=new_area)
			check=0.0;
			xr = sol1;
			pr = pa;   
			req_area = req_area - (xr-xa)*pa + new_area;
			tmpprob(k+1) = pr;
			tmpgain(k+1) = xr;    
			k = k+1;
			if (xr>=xb) 
				i=i+1;
			end;
		end;
	else
		tantheta=(pb-pa)/(xb-xa);
		c(1)=0.5*tantheta;
		c(2)=pa-xa*tantheta;
		c(3)=0.5*xa^2*tantheta-xa*pa-req_area;
		if (c(2)^2>4*c(1)*c(3)) 
			root=roots(c);
			sol1=root(1);
			sol2=root(2);
			if (check>=new_area)
				if ((pa+(sol1-xa)*tantheta)<0.0)
					sol1=sol2;
				elseif ((pa+(sol2-xa)*tantheta)<0.0)
					sol2=sol1;
				end;
				if ((sol1>xr) & (sol1>=xa) & (sol1<=xb))
					sol2=sol1;
				else
					sol1=sol2;
				end;
				check=0.0;
				xr=sol1;
				tmpgain(k+1)=xr;
				pr = pa+(xr-xa)*tantheta;
				tmpprob(k+1)=pr;
				req_area=req_area - 0.5*(xr-xa)*(pr+pa)+new_area;
				k=k+1;
				if (xr>=xb) 
					i=i+1;
				end;
			end;  
		end;
	end
end;
tmpgain(numregions+1)=gain(m+1);
tmpprob(numregions+1)=prob(m+1);
prob(1)=tmpprob(1);
gain(1)=tmpgain(1);
new_area=0;
for i=1:1:numregions, 
	prob(i+1)=tmpprob(i+1);
	gain(i+1)=tmpgain(i+1);
	new_area = new_area+0.5*(gain(i+1)-gain(i))*(prob(i)+prob(i+1));	
end;
%new_area = new_area/numregions;
%prob=prob/(numregions*new_area);
%new_area=area(gain,prob)
prob=prob./new_area;
%end;