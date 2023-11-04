%full state feedback design using estimator design
k1=7;
k2=4;
m=5;
b=2;
A=[0 1 0; -(k1+k2)/m 0 k1/m;k1/b 0 -k1/b]
B=[0; 1/m;0];
C=[0 0 1];
D=[0];
sys=ss(A,B,C,D)
Pc=ctrb(A,B);
Po=obsv(A,C);
%det
detPc=det(Pc);
detPo=det(Po);
if detPc==0
    disp('system is not controllable')
else
        disp("system is controllabe")
end
if detPo==0
    disp('system is not observable')
else
        disp("system is observable")
end
stepinfo(sys)
step(sys)
polDes=[-4+3*i;-4-3*i;-20];
Q=[-25 ; -26 ;-27];
% observer=>state measured
K=acker(A,B,polDes);
% K designed controller while L designed observer
L=acker(A',C',Q)'

% Response of states of observer design
Nbar=rscale(sys,K)

At=[A-B*K                       B*K
         zeros(size(A))         A-L*C];
Bt=[B*Nbar
        zeros(size(B))]
Ct=[C           zeros(size(C))]
sys_obs=ss(At,Bt,Ct,D)
t=0:0.01:2;
x0=[0.01 0 0];
x1=[0.2     0.2     0.2];
figure
lsim(sys_obs,zeros(size(t)),t,[x0 x1]);
title("Linear sim res with observer")
xlabel("Time")
ylabel("Position")
[y,t,x]=lsim(sys_obs,zeros(size(t)),t,[x0 x1])
n=3;
% Error
e=x(:,n+1:end);
% Actual
x=x(:,1:n);
x_est=x-e;

x1=x(:,1);x1_dot=x(:,2);z=x(:,3);
x1_est=x_est(:,1);x1_dot_est=x_est(:,2);z_est=x_est(:,3);
% closed loop form err
figure
subplot(2,3,1);
plot(t,x1,'-r',t,x1_est,':r');
legend('x1','x1_est')
subplot(2,3,2)
plot(t,x1_dot,'-b',t,x1_dot_est,':b');
legend('x1_dot','x1_dot_est')
subplot(2,3,3)
plot(t,z,'-g',t,z_est,':g');
legend('z','z_est')
% our goal to minmize settling time 
figure
sys_open=ss(A,B,C,D);
sys_obs=ss(At,Bt,Ct,D);
sys_closed=ss(A-B*K,B,C,D);
subplot(2,2,1)
step(sys_obs)
xlabel('Time (sec)')
ylabel('Sys-obser')

subplot(2,2,2)
step(sys_closed*Nbar)
xlabel('Time (sec)')
ylabel('Sys-closed')

subplot(2,2,3)
step(sys);
xlabel('Time (sec)')
ylabel('Step response open')