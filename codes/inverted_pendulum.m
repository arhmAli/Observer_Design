% Full state feedback using estimator design
clear all;
clc;
close all;
clearvars;
M = 0.5;  % mass of the cart
m = 0.2;  % mass of the pendulum
b = 0.1;  % coefficient of the friction for the cart
l = 0.3;  % length of the pendulum
I = 0.006; % moment of inertia
g = 9.8;   % gravity
p=(I*(M+m)+M*m*l^2);
A22=-(I+m*l^2)*b/p;
A23= (m^2*g*l^2)/(I*(M+m)+M*m*l^2);
A32= -(m*l*b)/(I*(M+m)+M*m*l^2);
A33= (m*g*l*(M+m))/(I*(M+m)+M*m*l^2);
B21= (I+m*l^2)/(I*(M+m)+M*m*l^2);
B41= (m*l)/(I*(M+m)+M*m*l^2);
A=[0 1 0 0;0 A22 A23 0;0 0 0 1;0 A32 A33 0];
B= [0;B21;0;B41];
C= [1   0   0   0];
D=[0];
sys = ss(A,B,C,D);

% To check controllability and observability
Pc = ctrb(A,B);
Po = obsv(A,C);
detPc = det(Pc);
detPo = det(Po);

if detPc == 0
    disp('System is not controllable')
else
    disp("System is controllable")
end

if detPo == 0
    disp('System is not observable')
else
    disp("System is observable")
end

% S = stepinfo(sys)
step(sys)
% poles
polDes = [-8+6i;-8-6i;-40;-40];
Q = [-51; -52; -53; -54];
K = acker(A,B,polDes);
L = acker(A',C',Q)';
% response of state of observer design
Nbar = rscale(sys,K);
At = [ A-B*K             B*K ;zeros(size(A))    A-L*C ];
Bt = [ B*Nbar ;zeros(size(B)) ];
Ct = [ C zeros(size(C)) ];
sys_ob = ss(At,Bt,Ct,0);
t = 0:0.01:2;
x0 = [0 0 0.005 0];
x1 = [0 0 0.005 0];
figure
lsim(sys_ob,zeros(size(t)),t,[x0 x1]);
title('Linear simulation results (with observer)')
xlabel('Time(sec)')
ylabel('Position(z)')
[y,t,x] = lsim(sys_ob,zeros(size(t)),t,[x0 x1]);
n = 4;
e = x(:,n+1:end); % X=actual state
x = x(:,1:n); 
x_est = x - e;
% Actual
x1 = x(:,1); x1_dot = x(:,2); z = x(:,3); x2 = x(:,4);
% Estimate
x1_est = x_est(:,1); x1_dot_est = x_est(:,2); z_est = x_est(:,3); x2_est = x_est(:,4);
figure
subplot(2,2,1);
plot(t,x1,'-r',t,x1_est,':r');
legend('x1','x1_{est}')
subplot(2,2,2);
plot(t,x1_dot,'-b',t,x1_dot_est,':b');
legend('x1dot','xqdot_{est}')
subplot(2,2,3);
plot(t,z,'-g',t,z_est,':g')
legend('z','z_{est}');
subplot(2,2,4);
plot(t,x2,'-m',t,x2_est,':m')
legend('x2','x2_{est}');
figure
sys_open = ss(A,B,C,D);
sys_obs = ss(At,Bt,Ct,D);
sys_closed = ss(A-B*K,B,C,D);
subplot(2,2,1)
step(sys_obs)
xlabel('Time (sec)')
ylabel('Sys-observer')

subplot(2,2,2)
step(sys_closed*Nbar)
xlabel('Time (sec)')
ylabel('Sys-closed')

subplot(2,2,3)
step(sys);
xlabel('Time (sec)')
ylabel('Step response open')
