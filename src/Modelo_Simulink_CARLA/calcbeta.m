function [costs,beta]=calcbeta(costs,jsim,jst_max);
% function [costs,beta]=calcbeta(costs,jsim,jst_max);
% Performance Evaluation Function 
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
    %jmed=mean(costs(:,1)); %assim ele não converge direito, demora ou nem vai, isso pq o jmedia pode ficar alto por causa de um outlier, ai todo jsim vai ficar abaixo, com reforço positivo
	jmin=min(costs(:,1));
	if (jsim>jmed)
		beta=0.0;
	elseif (jmed==jmin) %precisa disso pra não dar 0 no denominador, só
		beta =0;
	else
		%beta=(jmed-jsim)/(jmed-jmin);
        beta = min(max(0,((jmed-jsim)/(jmed-jmin))),1); %isso aqui significa a mesma coisa dessa logica acima
	end;
else 
	beta=0.0;
end;