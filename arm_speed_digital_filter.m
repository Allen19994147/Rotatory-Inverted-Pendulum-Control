%%%
clear all
clc

num = [3.14,-3.14];
den = [1,-0.7788];
num = [1 0];
G = tf(num,den,0.001)/4.5

%nyquist(G)
bode(G)

