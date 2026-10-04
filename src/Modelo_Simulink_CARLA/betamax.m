function [costs,beta]=betamax(costs,jsim,jst_max);
% function [costs,beta]=betamax(costs,jsim,jst_max);
% Performance Evaluation Function 
% for function minimization
if (jsim>0.0)
	[y,i]=max(costs(:,2));
	if (y==0) 
		i=1;
	end;
	costs(i,2)=0;
	i=i+1;
	if (i>jst_max)
		i=1;
	end;
	costs(i,2)=1;
	costs(i,1)=jsim;
	jmed=median(costs(:,1));
	jmax=max(costs(:,1));
   if (jsim>jmed)
 		beta=(jsim - jmed)/(jmax-jmed);
%		beta=0.0;
%	elseif (jmed==jmin)
%		beta =0;
	else
   	beta =0.0;
%		beta=(jmed-jsim)/(jmed-jmin);
	end;
else 
	beta=0.0;
end;