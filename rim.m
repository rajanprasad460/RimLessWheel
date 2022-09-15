function out=rim(t,x,m,mw,M,ang_inr,lwr,nw,g,l,T)
a=x(1);
a1=x(1)*ones(1,nw);
t(1)=x(2);
P=m*l*g*sin(a)+mw*sum(lwr.*sin(a1-ang_inr))*g;
t(2)=(-P+T)/M;
out=[t(1);t(2)];
end

