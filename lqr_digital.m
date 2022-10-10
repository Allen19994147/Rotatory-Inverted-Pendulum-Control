%%% Inverted pendulum controller design
%%% using LQR in z domain
clc
clear all
%______linearization plant___________
A = [0 0 1 0;0 0 0 1;0 -7.1247 -0.0963 0;0 59.3267 0.0741 0];
B = [0;0;7.0002;-5.3847];
C = [0 1 0 0];
D = [0];
%[num den] = ss2tf(A,B,C,[0])

%Rank = rank(ctrb(A,B));
T = 0.001;
[Ad Bd] = c2d(A,B,T);
[numz denz] = ss2tf(Ad,Bd,C,[0]);
Gz = tf(numz,denz,T);
%rlocus(Gz)
%Rank = rank(ctrb(Ad,Bd));
%__________K_____________________
R = [1];
Q = diag([12 25 9 10]);

Kd = dlqr(Ad,Bd,Q,R)
k1 = Kd(1,1);
k2 = Kd(1,2);
k3 = Kd(1,3);
k4 = Kd(1,4);
poles = eig(Ad-Bd*Kd)
N= [0];

% %_______compensated_________________
den_cmp = poly(Ad-Bd*Kd);
cmp = tf(numz,den_cmp,T)
% [num1 den1] = ss2tf(Ad-Bd*Kd,Bd,C,[0]);
% cmp2 = tf(num1,den1,T)
%%----control effort-------------------
[num7,den7] = ss2tf(Ad-Bd*Kd,Bd,1*Kd,[0]);
controller = tf(num7,den7,T);
Gw = d2c(cmp,'tustin');
rlocus(Gw)
sgrid
