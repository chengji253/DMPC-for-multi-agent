function [Aieq,bieq] = getAbieqHard(K,kci,A0,X,P,iagent,epsilon,sum_i,D,r_min,lambda)
 
Aieq = [-1*eye(3*K);eye(3*K)];
bieq = ones(6*K,1);
bieq(1:3*K) = 5*ones(3*K,1);
bieq(3*K+1:6*K) = 5*ones(3*K,1);
    

for i= 1:sum_i
     if D(iagent,i) == 1
          v_ij = P(3*kci(iagent , i)-2:3*kci(iagent , i),iagent)-P(3*kci(iagent , i)-2:3*kci(iagent , i),i);
          xi_ij = norm( v_ij);
          
          mu_ij = [zeros(3*(kci(iagent , i)-1),1)'  v_ij' zeros(3*(K-kci(iagent , i)),1)'  ]';
          rho = r_min*xi_ij + xi_ij^2 + v_ij'*P(3*kci(iagent , i)-2:3*kci(iagent , i),iagent);
         
          A = mu_ij'*lambda;
          b = rho - mu_ij'*A0*X(:,iagent) + epsilon* xi_ij;

          Aieq = [Aieq ; -A; ];
          bieq = [bieq ; -b; ];
     end
 end
%      %lambda_v = getLambda_v(K);
%     v_ij = P0(3*kci-2:3*kci)-P1(3*kci-2:3*kci);
%     xi_ij = norm( v_ij);
    
%     mu_ij = [zeros(3*(kci-1),1)'  v_ij' zeros(3*(K-kci),1)'  ]';
%     rho = r_min*xi_ij + xi_ij^2 + v_ij'*P0(3*kci-2:3*kci);
   
%     A = mu_ij'*lambda;
%     b = rho - mu_ij'*A0*X0 + epsilon* xi_ij;
   
%     Aieq = [Aieq ; -A; ];
%     bieq = [bieq ; -b; ];
