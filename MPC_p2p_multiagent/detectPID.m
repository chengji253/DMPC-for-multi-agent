function [Go] = detectPID(Pd,X,sum_i)

Go = zeros(sum_i,1);
 for i = 1:sum_i
    if norm(Pd(:,i) - X(1:3,i))<10
        Go(i) = 1;
    end
 end