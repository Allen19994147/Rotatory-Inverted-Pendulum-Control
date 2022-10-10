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

%Rank = rank(ctrb(Ac,Bc));
%__________K_____________________
R = [1];
Q = diag([12 20 5 5]);

Kc = lqr(A,B,Q,R)
k1 = Kc(1,1);
k2 = Kc(1,2);
k3 = Kc(1,3);
k4 = Kc(1,4);
poles = eig(A-B*Kc)
N= [0];

% %_______compensated_________________
den_cmp = poly(A-B*Kc);
cmp = tf(numz,den_cmp)
% [num1 den1] = ss2tf(Ad-Bd*Kc,Bd,C,[0]);
% cmp2 = tf(num1,den1,T)
%%----control effort-------------------
[num7,den7] = ss2tf(A-B*Kc,B,1*Kc,[0]);
controller = tf(num7,den7,T);


