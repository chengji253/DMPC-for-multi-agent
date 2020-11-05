function [D,kci] = detectCollision(P,K,r_min,sum_i)

D = zeros(sum_i,sum_i);   
kci = zeros(sum_i,sum_i);

% Time Complexity here is O(N^3),but sum of agent will not be very large.So it is acceptable.
% I think the algorithm here can be optimized. 
% if D = 0 no collision
% if D = 1 detect collision D(i,j) means i detect j 
for i = 1:sum_i
    for i1 = 1:sum_i
        if i ~=i1
            for kci_i = 1:K
                if(  norm(P(3*kci_i-2:3*kci_i,i)-P(3*kci_i-2:3*kci_i,i1))<r_min   )
                    D(i,i1) = 1; 
                    kci(i,i1) = kci_i;                   
                elseif kci == K
                    D(i,i1) = 0;
                    kci(i,i1) = 0;
                end
            end
        end
    end
end


  

