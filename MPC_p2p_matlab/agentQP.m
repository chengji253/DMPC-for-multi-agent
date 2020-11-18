function [u] = agentQP(ui,X,Pd,K,lambda,Q,delta,S,R,A0,Aieq,bieq)

Ustar = getUstar(ui,K); 

%---Quadratic Programming

H = (lambda'*Q*lambda)+(delta'*S*delta)+R;
f = - (Pd'*Q*lambda-(A0*X)'*Q*lambda)-(Ustar'*S*delta);

u = quadprog(H,f,Aieq,bieq);
