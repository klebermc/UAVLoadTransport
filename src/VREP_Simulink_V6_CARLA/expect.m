function [v,e]=expect(gain,prob);
%function [v,e]=expect(gain,prob);
% returns the expected value of the distribution
% and the variance of the distribution
% for use with th carla
%1. determine a list of m & c values for each section
[numregions,tmp]=size(gain);
numregions=numregions-1;
m = zeros(1,numregions);
c = zeros(1,numregions);
s1=0.0;
s2=0.0;
ss1=0.0;
ss2=0.0;
isqrdgain = gain(1)*gain(1);
icubedgain = isqrdgain*gain(1);
for i=1:numregions,
	xa=gain(i);
	pa=prob(i);
	xb=gain(i+1);
	pb=prob(i+1);
	if (pb==pa),
		m(i)=0;
		c(i)=pa;
	else
%		calculate gradient		
		m(i)=(pb-pa)/(xb-xa);
		c(i)=pa-m(i)*xa;
	end;
	i1sqrdgain = gain(i+1)*gain(i+1);
	i1cubedgain = i1sqrdgain*gain(i+1);
%	isqrdgain = gain(i)*gain(i);
%	icubedgain = isqrdgain*gain(i);
	s1 = s1 + 2*m(i)*i1cubedgain+3*c(i)*i1sqrdgain;
	s2 = s2 + 2*m(i)*icubedgain+3*c(i)*isqrdgain;
	ss1 = ss1 + i1cubedgain*( 3*m(i)*gain(i+1)+4*c(i) );
	ss2 = ss2 + icubedgain*( 3*m(i)*gain(i)+4*c(i) );
	% for the next iteration
	isqrdgain = i1sqrdgain;
	icubedgain = i1cubedgain;

end;
e=(s1-s2)/6;
vv=(ss1-ss2)/12;
v= vv - e*e;
