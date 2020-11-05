function [u] = PIDcontrol(Pd,X,iagent,error_sum,h)
% I find that utilizing PID in the process of convergence to the target
% will obtain better performance. 
kp = 1;
ki = 1;
kd = 1;

error =  Pd(:,iagent)-X(1:3,iagent);
error_sum(1:3,iagent) = error_sum(1:3,iagent)+error;



u = kp*error + ki*error_sum(1:3,iagent)+kd*(error-error_sum(4:6,iagent) );

error_sum(4:6,iagent) = error;

u1 = -X(4:6,iagent)/h/2;
u = u1+u;