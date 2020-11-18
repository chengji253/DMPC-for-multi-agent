function [Pd_sum_step] = changeDestination(Pd_Consensus,Pd_sum_step,sum_i)
    
Pd_sum_step(:,2:sum_i) = Pd_Consensus(:,2:sum_i);