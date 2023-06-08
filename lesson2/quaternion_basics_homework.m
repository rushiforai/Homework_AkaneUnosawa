close all;
clear all;

include_namespace_dq

%% Problem2
r1 = cos(pi/8) + i_*sin(pi/8);
%plot(r1)

%% Problem3
r2 = cos(pi/16) + j_*sin(pi/16);
%plot(r2)

%% Problem4
r3 = cos(pi/32) + k_*sin(pi/32);
%plot(r3)

%% Problem5
include_namespace_dq
disp('Displaying the result of the sequential multiplication r4 =r1*r2*r3')
r4 = r1*r2*r3
plot(r4)

%% Problem6
r5 = conj(r4)

%% Problem7
tmp = cos(2*pi) + i_*sin(2*pi)
r6 = r5 * tmp

plot(r5)
hold on
plot(r6)
%Is r5=-r6? --> No,it isn't.

