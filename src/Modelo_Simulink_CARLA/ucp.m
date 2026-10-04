function [gain,prob]=ucp(selected,beta,gain,prob,gw,gh);
% functon[gain,prob]=ucp(selected,beta,gain,prob,gw,gh);
% Continuous Learning Automata 
% Free Parameters
% Gaussian width 0.02;
% Gaussian height 0.3;
%
% 
% Copyright 1996/1997. Dr Mark Howell. Loughborough University
if (nargin==4)
	gw = 0.02;
	gh = 0.3;
end;
[numregions,tmp]=size(gain);
tmpprob=zeros(numregions,1);
tmpgain=zeros(numregions,1);
numregions=numregions-1;
sigma_rate = gw*(gain(numregions+1)-gain(1));
lambda = gh/(gain(numregions+1)-gain(1));
sqrsigma=sigma_rate^2;
prob=prob+beta*lambda*exp(-(gain-selected).^2/(2*sqrsigma));
new_area=0;
for i=1:1:numregions,
	new_area = new_area+0.5*(gain(i+1)-gain(i))*(prob(i)+prob(i+1));
end;
new_area=new_area/numregions;
tmpprob(1)=prob(1);
tmpgain(1)=gain(1);
xr=gain(1);
pr=prob(1);
req_area=new_area;
i=1;
k=1;
check=0.0;
while ((i<=numregions) & (k<=numregions))
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
		else
			%addition 18/12/97
			sol1=-c(2)/(2*c(1));
			if (check>=new_area)
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
			%addition 18/12/97
		end;
	end
end;
tmpgain(numregions+1)=gain(numregions+1);
tmpprob(numregions+1)=prob(numregions+1);
prob(1)=tmpprob(1);
gain(1)=tmpgain(1);
new_area=0;
for i=1:1:numregions, 
	prob(i+1)=tmpprob(i+1);
	gain(i+1)=tmpgain(i+1);
	new_area = new_area+0.5*(gain(i+1)-gain(i))*(prob(i)+prob(i+1));	
end;
prob = prob/new_area;
