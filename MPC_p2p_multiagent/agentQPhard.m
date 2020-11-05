function [u] = agentQPhard(ui,Pd,X,P,kci,D,K,lambda,Q,delta,S,R,A0,epsilon,r_min,sum_i,iagent)
                          
    Ustar = getUstar(ui,K); 
    %---Quadratic Programming
    
    
    H = (lambda'*Q*lambda)+(delta'*S*delta)+R;
    f = - (Pd'*Q*lambda-(A0*X(:,iagent))'*Q*lambda)-(Ustar'*S*delta);

    [Aieq,bieq] = getAbieqHard(K,kci,A0,X,P,iagent,epsilon,sum_i,D,r_min,lambda);
    
    u = quadprog(H,f,Aieq,bieq );
    % + k2*0.5*C*lambda