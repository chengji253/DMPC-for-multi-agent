function [j1] = judgeArrived(Pd,X,sum_i)

J = zeros(sum_i,1);
for i = 1:sum_i 
    if norm(Pd(1:3,i)-X(1:3,i))<=0.3 
        J(i,1) = 1;
    else
        J(i,1) = 0;
    end
end

if J == ones(sum_i,1)
    j1 = 1;
else
    j1 = 0;
end