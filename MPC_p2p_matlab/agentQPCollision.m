function [u] = agentQPCollision(ui,Pd,X,P,kci,D,K,lambda,Q,delta,S,R,A0,epsilon,r_min,sum_i,iagent)
          
    % Soft constraints
    Ustar = getUstar(ui,K); 
    C = zeros(3,3*K);
    H1 = zeros(3*K,3*K);
    f1 = zeros(1,3*K);
    Pd1 = [];
%     for i = 1:K
%         Pd1 = [Pd1;Pd];
%     end
%     Pd = Pd1;
    %---Quadratic Programming
    for i= 1:sum_i
        if D(iagent , i) == 1
            kci_i = kci(iagent , i);
            C(1,3*kci_i-2) = 1; C(2,3*kci_i-1) = 1; C(3,3*kci_i) = 1;
            Perror = P(kci_i:kci_i+2,iagent)-P(kci_i:kci_i+2,i); 
            %k1 = 1000000001000/(norm(Perror-r_min)+1);
            k1 = 10000001000/(norm(Perror-r_min)+1);

            H1 = H1 + k1*(lambda'*lambda);
            f1 = f1 + k1*(X'*A0'*C'*C*lambda-P(kci_i:kci_i+2,i)'*C*lambda);
        end
    end
    H = (lambda'*Q*lambda)+(delta'*S*delta)+R+H1;
    f = - (Pd'*Q*lambda-(A0*X)'*Q*lambda)-(Ustar'*S*delta)+f1;

    [Aieq,bieq] = getAbieaCollision(K);
    
    u = quadprog(H,f,Aieq,bieq );
    % + k2*0.5*C*lambda
