function [Pd_sum_step] = changeOnedes(Pd_sum_step ,pd,iagent,K)


for i = 1:K
    Pd_sum_step(3*i-2:3*i,iagent) = pd;
end
