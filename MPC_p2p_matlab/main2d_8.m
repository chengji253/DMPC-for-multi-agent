close all;clc;

sum_i = 8;  %sum_i of agents
K = 15;
h = 0.2;
epsilon = 0.05;
r_min = 3;
timesteps = 200; 

D = zeros(sum_i,sum_i);   % judgeFlag whether soft constraints
kci = zeros(sum_i,sum_i); %each kci of every agent .0 means no detected 
 
%----------some basic matrix ---------------
U = zeros(3*K,1); % ax0,ay0,az0,......,ax9,ay9,az9
x = zeros(6,1);
A = eye(6);
A(1:3,4:6) = h*eye(3);
B = zeros(6,3);
B(1:3,1:3) = 0.5*(h^2)*eye(3);
B(4:6,1:3) = h*eye(3);
phi = zeros(3,6);
phi(1:3,1:3) = eye(3);
lambda = getLambda(K , phi,A,B);
delta = getdelta(K);
lambda_v = getLambda_v(K);
A0 = getA0(K ,phi,A);
v1 = [1 1 2];
theta = diag(v1);
Go = zeros(sum_i,1);

%weight matrix
Q = eye(3*K);
Q(1:3*K-3,1:3*K-3) = zeros(3*K-3,3*K-3); % whether more aggressive
%Q(1:3*K-6,1:3*K-6) = zeros(3*K-6,3*K-6);
%Q(1:3*K-9,1:3*K-9) = zeros(3*K-9,3*K-9);

R = eye(3*K);
S = eye(3*K);
%------------------------------ ---------------

error_sum = zeros(6,sum_i); %record sum and error at present 

% formation state
ui = zeros(3,sum_i);  %initial input 
X = zeros(6,sum_i);  %initial state x y z vx vy vz
Pd = zeros(3,sum_i); %destination xd yd zd

% agent 1
ui(1:3,1) = zeros(1,3)';   %initial input
X(1:6,1) = [0 0 0 0 0 0]'; %initial state
Pd(1:3,1) = [100 50 0]';      %destination

%agent 2
ui(1:3,2) = zeros(1,3)';
X(1:6,2) = [0 50 0 0 0 0]';
Pd(1:3,2) = [100 0 0]';

%agent 3
ui(1:3,3) = zeros(1,3)';
X(1:6,3) = [50 0 0 0 0 0]';
Pd(1:3,3) = [50 50 0]';

%agent 4
ui(1:3,4) = zeros(1,3)';
X(1:6,4) = [100 0 0 0 0 0]';
Pd(1:3,4) = [0 50 0]';

%agent 5
ui(1:3,5) = zeros(1,3)';
X(1:6,5) = [50 50 0 0 0 0]';
Pd(1:3,5) = [50 0 0]';


%agent 6
ui(1:3,6) = zeros(1,3)';
X(1:6,6) = [100 50 0 0 0 0]';
Pd(1:3,6) = [0 0 0]';


%agent 7
ui(1:3,7) = zeros(1,3)';
X(1:6,7) = [100 25 0 0 0 0]';
Pd(1:3,7) = [0 25 0]';


%agent 8
ui(1:3,8) = zeros(1,3)';
X(1:6,8) = [0 25 0 0 0 0]';
Pd(1:3,8) = [100 25 0]';



P = zeros(3*K,sum_i);

% U constraints
Aieq = [-1*eye(3*K);eye(3*K)];
bieq = ones(6*K,1);
bieq(1:3*K) = 5*ones(3*K,1);
bieq(3*K+1:6*K) = 5*ones(3*K,1);
% 0:0.2:8  
%Jei = U'*(lambda'*Q*lambda)*U - 2*(Pd'*Q*lambda-(A0*X0)'*Q*lambda)*U;

%Jui = U'*R*U;

%Jdelta = U'*(delta'*S*delta)*U-2*(Ustar*S*delta)*U;

%J = Jei+Jui+Jdelta;
for iagent = 1:sum_i  %matrix store state and u
eval([ 'Xplot' num2str(iagent) '= [];' ]);
eval([ 'Uplot' num2str(iagent) '= [];' ]);
end

%

Pd_sum_step = zeros(3*K,sum_i);
%
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,1),1,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,2),2,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,3),3,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,4),4,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,5),5,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,6),6,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,7),7,K); %leader destination
[Pd_sum_step] = changeOnedes(Pd_sum_step,Pd(1:3,8),8,K); %leader destination



figure(1) 
% ---------------main loop-----------------
for t = 1:timesteps  
  for iagent = 1:sum_i  %for each agent
    if Go(iagent) == 1
        [u] = PIDcontrol(Pd,X,iagent,error_sum,h);       
    else  
        
      if sum( D(iagent,:) ) ~= 0
        
      u =  agentQPhard(ui(1:3,iagent),Pd_sum_step(:,iagent),X, ...
      P,kci,D,K,lambda,Q,delta,S,R,A0,epsilon,r_min,sum_i,iagent ); 
      if size(u,1) == 0  
        u = agentQPCollision(ui(1:3,iagent),Pd_sum_step(:,iagent),X(1:6,iagent), ...
        P,kci,D,K,lambda,Q,delta,S,R,A0,epsilon,r_min,sum_i,iagent );  
      end
    elseif sum( D(iagent,:) ) == 0
       
      u = agentQP(ui(1:3,iagent),X(1:6,iagent),Pd_sum_step(:,iagent) ,K,lambda,Q,delta,S,R,A0,Aieq,bieq);
    
      end
    end
    u_iagent = u(1:3); % MPC:using the first optimization input 
    u_iagent(3) = 0;
    X(1:6,iagent) = forwardState(X(1:6,iagent),u_iagent,A,B);
    
    %scatter3(X(1,iagent),X(2,iagent),X(3,iagent),'.'); hold on;drawnow
    
    eval( [ 'Xplot' num2str(iagent) ' = [Xplot' num2str(iagent) ';transpose(X(1:6,' num2str(iagent) '))];'])
    eval( [ 'Uplot' num2str(iagent) ' = [Uplot' num2str(iagent) ';transpose(ui(1:3,' num2str(iagent) '))];'])
  
   if size(u,1) ~=3
    P(:,iagent) = A0*X(1:6,iagent) + lambda*u;
  end
  end  %end each agent
    
%   scatter3(X(1,1),X(2,1),X(3,1),'b.'); hold on;drawnow;
%   scatter3(X(1,2),X(2,2),X(3,2),'r>'); hold on;drawnow;
%   scatter3(X(1,3),X(2,3),X(3,3),'g^'); hold on;drawnow;
%   scatter3(X(1,4),X(2,4),X(3,4),'ko'); hold on;drawnow;
  scatter(X(1,1),X(2,1),'b.'); hold on;drawnow;
  scatter(X(1,2),X(2,2),'r>'); hold on;drawnow;
  scatter(X(1,3),X(2,3),'g^'); hold on;drawnow;
  scatter(X(1,4),X(2,4),'b.'); hold on;drawnow;
  scatter(X(1,5),X(2,5),'r>'); hold on;drawnow;
  scatter(X(1,6),X(2,6),'ko'); hold on;drawnow;
  scatter(X(1,7),X(2,7),'r>'); hold on;drawnow;
  scatter(X(1,8),X(2,8),'b.'); hold on;drawnow;
  axis([-20 120 -10 60])
  [D,kci] = detectCollision(P,K,r_min,sum_i);
 
  [Go] = detectPID(Pd,X,sum_i);
 
 
 
 
   % collisionIF(X,t);
    J1 = judgeArrived(Pd,X,sum_i);
    if J1 == 1
      break;
    end
  

end % end for
%----------------end loop----------------
dis12 = Xplot1(:,1:3) - Xplot2(:,1:3);
dis13 = Xplot1(:,1:3) - Xplot3(:,1:3);
dis14 = Xplot1(:,1:3) - Xplot4(:,1:3);
dis23 = Xplot2(:,1:3) - Xplot3(:,1:3);
dis24 = Xplot2(:,1:3) - Xplot4(:,1:3);
dis34 = Xplot3(:,1:3) - Xplot4(:,1:3);

dissum = zeros(size(Xplot1,1) , 6);
for i=1:size(Xplot1,1)
    dissum(i,1) = norm(dis12(i,:));
    dissum(i,2) = norm(dis13(i,:));
    dissum(i,3) = norm(dis14(i,:));
    dissum(i,4) = norm(dis23(i,:));
    dissum(i,5) = norm(dis24(i,:));
    dissum(i,6) = norm(dis34(i,:));
end

figure(2)
for i = 1:6
plot(dissum(:,i)); hold on;
end
legend({'12' '13' '14' '23' '24' '34'});